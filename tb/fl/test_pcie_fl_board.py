"""test_pcie_fl_board -- pcie_fl_zcu102 at the values a board build uses.

The target verilate_fl_board passes no parameter, so both LTSSMs run their
real timers and the Root Complex holds its first Configuration Request for
100 ms. This module runs test_pcie_fl's recipe check and its first bring-up,
unchanged, and nothing else: the event times it logs (FL_TIMES) are the cycle
counts to expect in vio_fl on the board.
"""
from test_pcie_fl import (  # noqa: F401
    fl_debug_recipe_matches_the_bench_cores,
    fl_trains_and_enumerates_after_reset_release,
)
