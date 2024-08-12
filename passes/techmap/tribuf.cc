/*
 *  yosys -- Yosys Open SYnthesis Suite
 *
 *  Copyright (C) 2012  Claire Xenia Wolf <claire@yosyshq.com>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "kernel/rtlil.h"
#include "kernel/sigtools.h"

USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN

struct TribufConfig {
	bool merge_mode;
	bool logic_mode;
	bool formal_mode;
	bool propagate;

	TribufConfig()
	{
		merge_mode = false;
		logic_mode = false;
		formal_mode = false;
		propagate = false;
	}
};

struct TribufWorker {
	Module *module;
	SigMap sigmap;
	const TribufConfig &config;
	pool<SigBit> output_bits;

	TribufWorker(Module *module, const TribufConfig &config) : module(module), sigmap(module), config(config) {}

	static bool is_all_z(SigSpec sig)
	{
		for (auto bit : sig)
			if (bit != State::Sz)
				return false;
		return true;
	}

	void run()
	{
		// SigSpecs that are outputs of tri-state buffers
		pool<SigSpec> tribuf_signals;

		// SigSpecs that are inputs to mux or tri-state buffers cells
		dict<SigSpec, vector<Cell *>> know_muxes;

		// find all SigBits that are output ports
		if (config.logic_mode || config.formal_mode)
			for (auto wire : module->wires())
				if (wire->port_output)
					for (auto bit : sigmap(wire))
						output_bits.insert(bit);

		// find all cells that drive a signal
		dict<SigSpec, pool<Cell *>> driving_cells;
		for (auto cell : module->selected_cells())
			for (auto &conn : cell->connections())
				if (cell->output(conn.first))
					driving_cells[sigmap(conn.second)].insert(cell);

		for (auto cell : module->selected_cells()) {
			if (cell->type == ID($tribuf)) {
				tribuf_signals.insert(sigmap(cell->getPort(ID::Y)));
				know_muxes[sigmap(cell->getPort(ID::A))].push_back(cell);
			}

			if (cell->type == ID($_TBUF_)) {
				tribuf_signals.insert(sigmap(cell->getPort(ID::Y)));
				know_muxes[sigmap(cell->getPort(ID::A))].push_back(cell);
			}

			if (cell->type.in(ID($mux), ID($_MUX_))) {
				IdString en_port = cell->type == ID($mux) ? ID::EN : ID::E;
				IdString tri_type = cell->type == ID($mux) ? ID($tribuf) : ID($_TBUF_);

				bool is_a_all_z = is_all_z(cell->getPort(ID::A));
				bool is_b_all_z = is_all_z(cell->getPort(ID::B));

				if (config.propagate && !is_a_all_z && !is_b_all_z) {
					know_muxes[sigmap(cell->getPort(ID::A))].push_back(cell);
					know_muxes[sigmap(cell->getPort(ID::B))].push_back(cell);
				}

				if (is_a_all_z && is_b_all_z) {
					module->remove(cell);
					driving_cells.at(sigmap(cell->getPort(ID::Y))).erase(cell);
					continue;
				}

				if (is_a_all_z) {
					cell->setPort(ID::A, cell->getPort(ID::B));
					cell->setPort(en_port, cell->getPort(ID::S));
					cell->unsetPort(ID::B);
					cell->unsetPort(ID::S);
					cell->type = tri_type;
					tribuf_signals.insert(sigmap(cell->getPort(ID::Y)));
					if (config.propagate)
						know_muxes[sigmap(cell->getPort(ID::A))].push_back(cell);
					module->design->scratchpad_set_bool("tribuf.added_something", true);
					continue;
				}

				if (is_b_all_z) {
					cell->setPort(en_port, module->Not(NEW_ID, cell->getPort(ID::S)));
					cell->unsetPort(ID::B);
					cell->unsetPort(ID::S);
					cell->type = tri_type;
					tribuf_signals.insert(sigmap(cell->getPort(ID::Y)));
					if (config.propagate)
						know_muxes[sigmap(cell->getPort(ID::A))].push_back(cell);
					module->design->scratchpad_set_bool("tribuf.added_something", true);
					continue;
				}
			}
		}

		if (config.propagate) {
			// SigSpecs that are outputs of a tri-state buffers and still need to
			// be checked for propagation
			pool<SigSpec> newly_added_tribufs;

			for (auto &it : tribuf_signals) {
				newly_added_tribufs.insert(it);
			}
			pool<SigSpec> muxes_to_check;

			while (!newly_added_tribufs.empty()) {
				log("Propagating tri-state buffers through muxes: %d muxes left.\n", int(newly_added_tribufs.size()));
				std::swap(newly_added_tribufs, muxes_to_check);
				newly_added_tribufs.clear();
				for (auto it : muxes_to_check) {
					if (know_muxes.count(it) == 0)
						continue;

					// check that all driving cells are tri-state buffers
					for (const auto &cell : driving_cells.at(it)) {
						if (cell->type != ID($tribuf) && cell->type != ID($_TBUF_)) {
							log("There is a non-tri-state buffer driving %s\n", log_signal(it));
							continue;
						}
					}

					if (driving_cells.at(it).size() > 1)
						merge(it, driving_cells.at(it));

					if (tribuf_signals.count(it) != 1) {
						log_warning("No tribuf for %s\n", log_signal(it));
						continue;
					}

					RTLIL::Cell *tribuf = *driving_cells.at(it).begin();
					tribuf_signals.erase(it);

					IdString en1_port = tribuf->type == ID($tribuf) ? ID::EN : ID::E;

					// propagates the tribuf that drives this signal through all
					// muxes/tribufs that this signal drives
					for (auto cell : know_muxes.at(it)) {
						if (cell->type == ID($mux) || cell->type == ID($_MUX_)) {
							IdString en2_port = cell->type == ID($mux) ? ID::EN : ID::E;
							IdString tri_type = cell->type == ID($mux) ? ID($tribuf) : ID($_TBUF_);

							if (sigmap(cell->getPort(ID::A)) == it) {
								// convert: $tribuf(A, E, Y) -> $mux(Y, B, S, Y2)
								//      to:                     $mux(A, B, S, Y3) -> $tribuf(Y3, E || S, Y2)
								const RTLIL::SigSpec &y2 = cell->getPort(ID::Y);
								RTLIL::Wire *y3 = module->addWire(NEW_ID, GetSize(tribuf->getPort(ID::Y)));

								cell->setPort(ID::A, tribuf->getPort(ID::A));
								RTLIL::Cell *new_tribuf = module->addTribuf(
								  NEW_ID, y3, module->Or(NEW_ID, tribuf->getPort(en1_port), cell->getPort(ID::S)),
								  y2);

								{
									RTLIL::SigSpec sy2 = sigmap(y2);
									tribuf_signals.insert(sy2);
									newly_added_tribufs.insert(sy2);
									driving_cells.at(sy2).insert(new_tribuf);
								}

								cell->setPort(ID::Y, y3);
							} else if (sigmap(cell->getPort(ID::B)) == it) {
								// convert: $tribuf(A, E, Y) -> $mux(C, Y, S, Y2)
								//      to:                     $mux(C, A, S, Y3) -> $tribuf(Y3, E || ~S, Y2)
								const RTLIL::SigSpec &y2 = cell->getPort(ID::Y);
								RTLIL::Wire *y3 = module->addWire(NEW_ID, GetSize(tribuf->getPort(ID::Y)));

								cell->setPort(ID::B, tribuf->getPort(ID::A));
								RTLIL::Cell *new_tribuf =
								  module->addTribuf(NEW_ID, y3,
										    module->Or(NEW_ID, tribuf->getPort(en1_port),
											       module->Not(NEW_ID, cell->getPort(ID::S))),
										    y2);

								{
									RTLIL::SigSpec sy2 = sigmap(y2);
									tribuf_signals.insert(sy2);
									newly_added_tribufs.insert(sy2);
									driving_cells.at(sy2).insert(new_tribuf);
								}

								cell->setPort(ID::Y, y3);
							} else
								log_warning("Mux %s is not driven by %s, but %s or %s\n", log_id(cell),
									    log_signal(it), log_signal(cell->getPort(ID::A)),
									    log_signal(cell->getPort(ID::B)));
						}
						if (cell->type == ID($tribuf) || cell->type == ID($_TBUF_)) {
							IdString en2_port = cell->type == ID($tribuf) ? ID::EN : ID::E;

							if (sigmap(cell->getPort(ID::A)) == it) {
								// convert: $tribuf(A, E1, Y) -> $tribuf(Y, E2, Y2)
								//      to:                      $tribuf(A, E1 && E2, Y2)
								cell->setPort(ID::A, tribuf->getPort(ID::A));
								cell->setPort(
								  en2_port, module->And(NEW_ID, tribuf->getPort(en1_port), cell->getPort(en2_port)));
							} else {
								log_warning("Tri-state %s is not driven by %s, but %s\n", log_id(cell),
									    log_signal(it), log_signal(cell->getPort(ID::A)));
							}
						}
					}

					// module->remove(tribuf);
					// driving_cells.at(it).erase(tribuf);
				}
			}
		}

		if (config.merge_mode || config.logic_mode || config.formal_mode) {
			for (auto &it : tribuf_signals) {
				// check that all driving cells are tri-state buffers
				for (const auto &cell : driving_cells.at(it)) {
					if (cell->type != ID($tribuf) && cell->type != ID($_TBUF_)) {
						log("There is a non-tri-state buffer driving %s\n", log_signal(it));
						goto skip_signal;
					}
				}
				merge(it, driving_cells.at(it));
			skip_signal:;
			}
		}
	}

	void merge(RTLIL::SigSpec sig, pool<RTLIL::Cell *> &cells)
	{
		bool no_tribuf = false;

		if (config.logic_mode && !config.formal_mode) {
			no_tribuf = true;
			for (auto bit : sig)
				if (output_bits.count(bit))
					no_tribuf = false;
		}

		if (config.formal_mode)
			no_tribuf = true;

		if (GetSize(cells) <= 1 && !no_tribuf)
			return;

		if (config.formal_mode && GetSize(cells) >= 2) {
			for (auto cell : cells) {
				SigSpec others_s;

				for (auto other_cell : cells) {
					if (other_cell == cell)
						continue;
					else if (other_cell->type == ID($tribuf))
						others_s.append(other_cell->getPort(ID::EN));
					else
						others_s.append(other_cell->getPort(ID::E));
				}

				auto cell_s = cell->type == ID($tribuf) ? cell->getPort(ID::EN) : cell->getPort(ID::E);

				auto other_s = module->ReduceOr(NEW_ID, others_s);

				auto conflict = module->And(NEW_ID, cell_s, other_s);

				std::string name = stringf("$tribuf_conflict$%s", log_id(cell->name));
				auto assert_cell = module->addAssert(name, module->Not(NEW_ID, conflict), SigSpec(true));

				assert_cell->set_src_attribute(cell->get_src_attribute());
				assert_cell->set_bool_attribute(ID::keep);

				module->design->scratchpad_set_bool("tribuf.added_something", true);
			}
		}

		SigSpec pmux_b, pmux_s;
		for (auto cell : cells) {
			if (cell->type == ID($tribuf))
				pmux_s.append(cell->getPort(ID::EN));
			else
				pmux_s.append(cell->getPort(ID::E));
			pmux_b.append(cell->getPort(ID::A));
			module->remove(cell);
		}
		cells.clear();

		SigSpec muxout = GetSize(pmux_s) > 1 ? module->Pmux(NEW_ID, SigSpec(State::Sx, GetSize(sig)), pmux_b, pmux_s) : pmux_b;

		if (no_tribuf)
			module->connect(sig, muxout);
		else {
			RTLIL::Cell *new_tribuf = module->addTribuf(NEW_ID, muxout, module->ReduceOr(NEW_ID, pmux_s), sig);
			module->design->scratchpad_set_bool("tribuf.added_something", true);
			cells.insert(new_tribuf);
		}
	}
};

struct TribufPass : public Pass {
	TribufPass() : Pass("tribuf", "infer tri-state buffers") {}
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    tribuf [options] [selection]\n");
		log("\n");
		log("This pass transforms $mux cells with 'z' inputs to tristate buffers.\n");
		log("\n");
		log("    -merge\n");
		log("        merge multiple tri-state buffers driving the same net\n");
		log("        into a single buffer.\n");
		log("\n");
		log("    -logic\n");
		log("        convert tri-state buffers that do not drive output ports\n");
		log("        to non-tristate logic. this option implies -merge.\n");
		log("\n");
		log("    -formal\n");
		log("        convert all tri-state buffers to non-tristate logic and\n");
		log("        add a formal assertion that no two buffers are driving the\n");
		log("        same net simultaneously. this option implies -merge.\n");
		log("\n");
		log("    -propagate\n");
		log("        propagate the tribuffer through mux cells. Basically converts\n");
		log("        `x ? (y ? a : 1'bz) : b` into `y || ~x ? (x ? a : b) : 1'bz`,\n");
		log("        etc.\n");
		log("\n");
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		TribufConfig config;

		log_header(design, "Executing TRIBUF pass.\n");

		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++) {
			if (args[argidx] == "-merge") {
				config.merge_mode = true;
				continue;
			}
			if (args[argidx] == "-logic") {
				config.logic_mode = true;
				continue;
			}
			if (args[argidx] == "-formal") {
				config.formal_mode = true;
				continue;
			}
			if (args[argidx] == "-propagate") {
				config.propagate = true;
				config.merge_mode = true;
				continue;
			}
			break;
		}
		extra_args(args, argidx, design);

		for (auto module : design->selected_modules()) {
			TribufWorker worker(module, config);
			worker.run();
		}
	}
} TribufPass;

PRIVATE_NAMESPACE_END
