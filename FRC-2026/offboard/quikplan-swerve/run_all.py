from multiprocessing import Process
import os

import run_simple_test
import run_square_test
import run_crazy_test
import run_goal_test
import run_orbit_test
import run_first_cycle
import run_first_cycle_no_opp
import run_second_cycle
import run_second_cycle_inside_out
import run_second_cycle_outside_in
import run_second_cycle_no_citrus
import run_first_cycle_vs_XXXX
import run_vs_XXXX_XXXX_first_sweep
import run_vs_XXXX_XXXX_second_sweep
import run_turn_forward_transition
import run_feed_first_cycle_long
import run_feed_first_cycle_short
import run_feed_cleanup
import run_first_cycle_with_XXXX
import run_second_cycle_with_XXXX
import run_center_pause
import run_center_pause_sweep
import run_center_pause_sweep_opp
import run_center_pause_sweep_hub
import run_center_pause_slow
import run_center_pause_slow_sweep_hub
import run_shoot_to_depot
import run_shoot_to_depot_left
import run_go_after_partner
import run_go_after_partner_with_pause
import run_after_partner_return
import run_after_partner_return_opp
import run_after_partner_feed
import run_fast_race
import run_fast_race_return
import run_slow_race

if __name__ == "__main__":
    # Delete old plots and CSVs
    dirname = os.path.dirname(__file__)
    for item in os.listdir(os.path.join(dirname, "plots")):
        if item.endswith(".png"):
            os.remove(os.path.join(dirname, "plots", item))
    for item in os.listdir(os.path.join(dirname, "../../src/main/deploy")):
        if item.endswith(".csv"):
            os.remove(os.path.join(dirname, "../../src/main/deploy", item))

    # Test plans
    Process(target=run_simple_test.plan, args=(True,)).start()
    Process(target=run_square_test.plan, args=(True,)).start()
    Process(target=run_crazy_test.plan, args=(True,)).start()
    Process(target=run_goal_test.plan, args=(True,)).start()
    Process(target=run_orbit_test.plan, args=(True,)).start()

    # Autos
    Process(target=run_first_cycle.plan, args=(True,)).start()
    Process(target=run_first_cycle_no_opp.plan, args=(True,)).start()
    Process(target=run_second_cycle.plan, args=(True,)).start()
    Process(target=run_second_cycle_inside_out.plan, args=(True,)).start()
    Process(target=run_second_cycle_outside_in.plan, args=(True,)).start()
    Process(target=run_second_cycle_no_citrus.plan, args=(True,)).start()
    Process(target=run_first_cycle_vs_XXXX.plan, args=(True,)).start()
    Process(target=run_vs_XXXX_XXXX_first_sweep.plan, args=(True,)).start()
    Process(target=run_vs_XXXX_XXXX_second_sweep.plan, args=(True,)).start()
    Process(target=run_turn_forward_transition.plan, args=(True,)).start()

    Process(target=run_feed_first_cycle_long.plan, args=(True,)).start()
    Process(target=run_feed_first_cycle_short.plan, args=(True,)).start()
    Process(target=run_feed_cleanup.plan, args=(True,)).start()

    Process(target=run_first_cycle_with_XXXX.plan, args=(True,)).start()
    Process(target=run_second_cycle_with_XXXX.plan, args=(True,)).start()

    Process(target=run_center_pause.plan, args=(True,)).start()
    Process(target=run_center_pause_sweep.plan, args=(True,)).start()
    Process(target=run_center_pause_sweep_opp.plan, args=(True,)).start()
    Process(target=run_center_pause_sweep_hub.plan, args=(True,)).start()

    Process(target=run_center_pause_slow.plan, args=(True,)).start()
    Process(target=run_center_pause_slow_sweep_hub.plan, args=(True,)).start()
    Process(target=run_shoot_to_depot.plan, args=(True,)).start()
    Process(target=run_shoot_to_depot_left.plan, args=(True,)).start()

    Process(target=run_go_after_partner.plan, args=(True,)).start()
    Process(target=run_go_after_partner_with_pause.plan, args=(True,)).start()
    Process(target=run_after_partner_return.plan, args=(True,)).start()
    Process(target=run_after_partner_return_opp.plan, args=(True,)).start()
    Process(target=run_after_partner_feed.plan, args=(True,)).start()

    Process(target=run_fast_race.plan, args=(True,)).start()
    Process(target=run_fast_race_return.plan, args=(True,)).start()
    Process(target=run_slow_race.plan, args=(True,)).start()
