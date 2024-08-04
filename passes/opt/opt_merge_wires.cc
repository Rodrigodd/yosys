/*
 *  yosys -- Yosys Open SYnthesis Suite
 *
 *  Copyright (C) 2024  Rodrigo Batista de Moraes <rodrigobatsmoraes@hotmail.com>
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

#include "kernel/celltypes.h"
#include "kernel/ffinit.h"
#include "kernel/log.h"
#include "kernel/register.h"
#include "kernel/sigtools.h"
#include "kernel/yosys_common.h"
#include "libs/sha1/sha1.h"
#include <set>
#include <stdio.h>
#include <stdlib.h>

USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN

struct OptMergeWiresPass : public Pass {
	OptMergeWiresPass() : Pass("opt_merge_wires", "merge connected wires into one") {}
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    opt_merge_wires [options] [selection]\n");
		log("\n");
		log("This pass identifies any weakly connected component of wires, and create a\n");
		log("single wire representing this component, making any cell connect to any wire of\n");
		log("the component connect to that representative wire.\n");
		log("\n");
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		log_header(design, "Executing OPT_MERGE_WIRES pass (detect connected wires).\n");
		extra_args(args, 1, design);

		int changed_wires = 0;

		for (auto module : design->selected_modules()) {
			dict<RTLIL::Wire *, RTLIL::Wire *> wire_to_representative_wire;
			{
				// merge-find set of weakly connected component of wires
				mfp<RTLIL::Wire *> components;
				log("Finding weakly connected component of wires in module `%s'.\n", module->name.c_str());

				for (auto conn : module->connections()) {
					if (conn.first.is_wire() && conn.second.is_wire() && design->selected(module, conn.first.as_wire()) &&
					    design->selected(module, conn.second.as_wire())) {
						log("Merging wires `%s' and `%s'.\n", conn.first.as_wire()->name.c_str(),
						    conn.second.as_wire()->name.c_str());
						// if two wires are connected, they are from the same component
						components.merge(conn.first.as_wire(), conn.second.as_wire());
					}
				}

				// Map each wire to a representative wire. The choice of the
				// representative wire could be arbitrary, but we prefer a wire
				// that is a input port, or one that have a public name.
				dict<RTLIL::Wire *, pool<RTLIL::Wire *>> components_map;
				for (auto wire : components) {
					components_map[components.find(wire)].insert(wire);
				}
				for (auto component : components_map) {
					RTLIL::Wire *representative_wire = nullptr;
					for (auto wire : component.second) {
						if (wire->port_input) {
							representative_wire = wire;
							break;
						}
					}
					if (representative_wire == nullptr) {
						for (auto wire : component.second) {
							if (wire->name[0] != '$') {
								representative_wire = wire;
								break;
							}
						}
					}
					if (representative_wire == nullptr) {
						representative_wire = *component.second.begin();
					}
					for (auto wire : component.second) {
						if (wire == representative_wire)
							continue;
						wire_to_representative_wire[wire] = representative_wire;
					}
				}
			}

			// create a rule that map any sigbit of a wire to the sigbit of the representative wire
			dict<SigBit, SigBit> rules;
			for (auto pair : wire_to_representative_wire) {
				RTLIL::Wire *wire = pair.first;
				RTLIL::Wire *representative_wire = pair.second;

				changed_wires++;
				log("Mapping wire `%s' to representative wire `%s'.\n", wire->name.c_str(), representative_wire->name.c_str());
				for (int i = 0; i < GetSize(wire); i++) {
					rules[SigBit(wire, i)] = SigBit(representative_wire, i);
				}
			}

			// apply the rule to all cells connections
			for (auto cell : module->cells()) {
				for (auto conn : cell->connections()) {
					const char *from = log_signal(conn.second);
					conn.second.replace(rules);
					cell->setPort(conn.first, conn.second);
					const char *to = log_signal(conn.second);
					// if (strcmp(from, to) != 0) {
					log("Mapped cell `%s' port `%s' from `%s' to '%s'.\n", cell->name.c_str(), conn.first.c_str(), from, to);
					// }
				}
			}

			// any wire that is not a representative wire will not have any
			// outbounds connections, with a single inbound connection from the
			// representative wire.

			// first, remove all connections between wires
			module->connections_.erase(std::remove_if(module->connections_.begin(), module->connections_.end(),
								  [&](const std::pair<RTLIL::SigSpec, RTLIL::SigSpec> &conn) {
									  return conn.first.is_wire() && conn.second.is_wire() &&
										 design->selected(module, conn.first.as_wire()) &&
										 design->selected(module, conn.second.as_wire());
								  }),
						   module->connections_.end());

			// apply the rule to all non-selected connections
			for (auto &connections : module->connections_) {
				const char *first = log_signal(connections.first);
				const char *second = log_signal(connections.second);
				connections.first.replace(rules);
				connections.second.replace(rules);
				const char *first2 = log_signal(connections.first);
				const char *second2 = log_signal(connections.second);
				if (strcmp(first, first2) != 0 || strcmp(second, second2) != 0) {
					log("Mapped non-selected connection from `%s` <- `%s` to `%s` <- `%s`.\n", first, second, first2, second2);
				}
			}

			// then, connect the representative wire to each wire
			for (auto pair : wire_to_representative_wire) {
				RTLIL::Wire *wire = pair.first;
				RTLIL::Wire *representative_wire = pair.second;

				log("Connecting representative wire `%s' to wire `%s'.\n", representative_wire->name.c_str(), wire->name.c_str());
				module->connect(RTLIL::SigSig(wire, representative_wire));
			}

			log_suppressed();
		}

		if (changed_wires > 0)
			design->scratchpad_set_bool("opt.did_something", true);
		log("Modify a total of %d wires.\n", changed_wires);
	}
} OptMergeWiresPass;

PRIVATE_NAMESPACE_END
