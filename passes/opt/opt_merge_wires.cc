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
			dict<RTLIL::SigBit, RTLIL::SigBit> sigbit_to_representative_sigbit;
			SigMap sigmap;
			// exclude constants from the sigmap (otherwise they will merge
			// unrelelated components just because they are connected to the
			// same constant)
			for (auto &it : module->connections()) {
				for (int i = 0; i < GetSize(it.first); i++) {
					if (it.first[i].wire == nullptr || it.second[i].wire == nullptr)
						goto skip_connection;
				}
				sigmap.add(it.first, it.second);
			skip_connection:;
			}
			{

				// merge-find set of weakly connected component of wires

				log("Finding weakly connected component of sigbits in module `%s'.\n", module->name.c_str());

				// Map each sigbit to a representative sigbit. The choice of the
				// representative wire could be arbitrary, but we prefer a wire
				// that is a input port, or one that have a public name.
				dict<RTLIL::SigBit, pool<RTLIL::SigBit>> components_map;
				for (auto sigbit : sigmap.allbits()) {
					components_map[sigmap(sigbit)].insert(sigbit);
				}
				for (auto component : components_map) {
					RTLIL::SigBit representative_sigbit;
					for (auto sigbit : component.second) {
						if (sigbit.wire->port_input) {
							representative_sigbit = sigbit;
							break;
						}
					}
					if (representative_sigbit.wire == nullptr) {
						for (auto sigbit : component.second) {
							if (sigbit.wire->name[0] != '$') {
								representative_sigbit = sigbit;
								break;
							}
						}
					}
					if (representative_sigbit.wire == nullptr) {
						representative_sigbit = *component.second.begin();
					}

					log("Representative sigbit for component `%s' is `%s'.\n", log_signal(component.second),
					    log_signal(representative_sigbit));

					for (auto sigbit : component.second) {
						if (sigbit == representative_sigbit)
							continue;
						sigbit_to_representative_sigbit[sigbit] = representative_sigbit;
					}
				}
			}

			// apply the rule to all cells connections
			for (auto cell : module->cells()) {
				for (auto conn : cell->connections()) {
					const char *from = log_signal(conn.second);
					conn.second.replace(sigbit_to_representative_sigbit);
					cell->setPort(conn.first, conn.second);
					const char *to = log_signal(conn.second);
					// if (strcmp(from, to) != 0) {
					log("Mapped cell `%s' port `%s' from `%s' to '%s'.\n", cell->name.c_str(), conn.first.c_str(), from, to);
					// }
				}
			}

			// any sigbit that is not a representative sigbit will not have any
			// outbounds connections, with a single inbound connection from the
			// representative sigbit.

			// first, remove all connections between sigbits of the same component
			for (auto &connection : module->connections_) {
				SigSpec first = sigmap(connection.first);
				SigSpec second = sigmap(connection.second);

				// log("Modifying connection from `%s' <- `%s' to '%s' <- '%s'.\n", log_signal(connection.first),
				//     log_signal(connection.second), log_signal(first), log_signal(second));

				// remove sigbits connected to themselves
				bool changed = false;
				for (int i = GetSize(first) - 1; i >= 0; i--) {
					if (first[i] == second[i]) {
						connection.first.remove(i);
						connection.second.remove(i);
						changed = true;
					}
				}
				// log("Becames `%s' <- `%s'.\n", log_signal(connection.first), log_signal(connection.second));
				if (changed) {
					changed_wires++;
				}
			}
			// remove connections that become empty
			module->connections_.erase(std::remove_if(module->connections_.begin(), module->connections_.end(),
								  [&](const std::pair<RTLIL::SigSpec, RTLIL::SigSpec> &conn) {
									  log_assert(conn.first.size() == conn.second.size());
									  return conn.first.empty();
								  }),
						   module->connections_.end());

			// any wire that was driving a component wire will now be driven by the representative wire
			for (auto &connections : module->connections_) {
				const char *first = log_signal(connections.first);
				const char *second = log_signal(connections.second);
				connections.first.replace(sigbit_to_representative_sigbit);
				connections.second.replace(sigbit_to_representative_sigbit);
				const char *first2 = log_signal(connections.first);
				const char *second2 = log_signal(connections.second);
				if (strcmp(first, first2) != 0 || strcmp(second, second2) != 0) {
					log("Mapped non-selected connection from `%s` <- `%s` to `%s` <- `%s`.\n", first, second, first2, second2);
				}
			}

			// then, connect the representative sigbit to each sigbit
			// but first, try to pack the sigbits in sigspecs

			// convert dict in to vector<pair>
			std::vector<std::pair<RTLIL::SigSpec, RTLIL::SigSpec>> sigbit_to_representative_sigbit_vector;
			for (auto pair : sigbit_to_representative_sigbit) {
				sigbit_to_representative_sigbit_vector.push_back(pair);
			}

			// sort the vector by the representative sigbit
			std::sort(sigbit_to_representative_sigbit_vector.begin(), sigbit_to_representative_sigbit_vector.end(),
				  [](const std::pair<RTLIL::SigSpec, RTLIL::SigSpec> &a, const std::pair<RTLIL::SigSpec, RTLIL::SigSpec> &b) {
					  return a.first[0].wire != b.first[0].wire ? a.first[0].wire < b.first[0].wire
										    : a.second[0].offset < b.second[0].offset;
				  });

			// merge consecutive sigbits in sigspecs if they connect the same wires
			if (GetSize(sigbit_to_representative_sigbit_vector) > 1) {
				int i, j;
				for (i = 0, j = 1; j < GetSize(sigbit_to_representative_sigbit_vector); j++) {
					bool first_same = sigbit_to_representative_sigbit_vector[i].first[0].wire ==
							  sigbit_to_representative_sigbit_vector[j].first[0].wire;
					bool second_same = sigbit_to_representative_sigbit_vector[i].second[0].wire ==
							   sigbit_to_representative_sigbit_vector[j].second[0].wire;
					// log("Comparing `%s' == `%s' and `%s' == `%s'.\n",
					// log_signal(sigbit_to_representative_sigbit_vector[i].first),
					//     log_signal(sigbit_to_representative_sigbit_vector[j].first),
					//     log_signal(sigbit_to_representative_sigbit_vector[i].second),
					//     log_signal(sigbit_to_representative_sigbit_vector[j].second));

					if (first_same && second_same) {
						sigbit_to_representative_sigbit_vector[i].first.append(
						  sigbit_to_representative_sigbit_vector[j].first);
						sigbit_to_representative_sigbit_vector[i].second.append(
						  sigbit_to_representative_sigbit_vector[j].second);
						continue;
					}

					i++;
					if (i != j)
						sigbit_to_representative_sigbit_vector[i] = sigbit_to_representative_sigbit_vector[j];
				}
				sigbit_to_representative_sigbit_vector.resize(i + 1);
				log("Merged %d sigbits in %d sigspecs:\n", j, i + 1);
				for (int k = 0; k <= i; k++) {
					log("  `%s' -> `%s'.\n", log_signal(sigbit_to_representative_sigbit_vector[k].first),
					    log_signal(sigbit_to_representative_sigbit_vector[k].second));
				}
			}

			for (auto pair : sigbit_to_representative_sigbit_vector) {
				RTLIL::SigSpec sigbit = pair.first;
				RTLIL::SigSpec representative_sigbit = pair.second;

				log("Connecting representative sigbit `%s' to wire `%s'.\n", log_signal(representative_sigbit), log_signal(sigbit));
				module->connect(RTLIL::SigSig(sigbit, representative_sigbit));
			}

			log_suppressed();
		}

		if (changed_wires > 0)
			design->scratchpad_set_bool("opt.did_something", true);
		log("Modify a total of %d connections.\n", changed_wires);
	}
} OptMergeWiresPass;

PRIVATE_NAMESPACE_END
