// ---------------------------------------------------------------------------
// pcie_datalink_init -- Data Link Control and Management State Machine
//
//! @title pcie_datalink_init
//! @author Idris Somoye
//
// Purpose
//   Tracks the Data Link Layer state from Physical LinkUp and from the flow
//   control initialization progress, and reports it on link_status_o. Starts
//   flow control initialization on link-up, and holds the rest of the Data
//   Link Layer in reset while the link is down.
//
// Interfaces
//   Link          phy_link_up_i: Physical LinkUp; its loss returns every
//                 state to ST_DL_INACTIVE.
//   Control       init_flow_control_o: start_flow_control_i of
//                 pcie_flow_ctrl_init; rises on the first link-up and stays
//                 high until rst_i. soft_reset_o: high in ST_DL_INACTIVE;
//                 pcie_datalink_layer ORs it into its submodules' resets.
//   Flow control  init_ack_i: pcie_flow_ctrl_init has left ST_IDLE.
//                 fc1_values_stored_i, fc2_values_stored_i: the peer's
//                 InitFC1 or InitFC2 values for P, NP and Cpl are all stored
//                 (dllp_handler).
//   Status        link_status_o: DL_DOWN until ST_DL_INIT_FC2, DL_UP there,
//                 DL_ACTIVE in ST_DL_ACTIVE, where the specification reports
//                 DL_Up.
//
// Clock and reset
//   clk_i only. rst_i is asynchronous and active high.
//
// Limitations
//   ST_DL_ACTIVE is entered on fc2_values_stored_i alone; this module does
//   not see whether pcie_flow_ctrl_init has sent its InitFC2 set. There is no
//   link-disable input: ST_DL_INACTIVE exits on Physical LinkUp alone.
//   link_state is not used.
//
// References
//   PCIe Base Spec r2.1, §3.2
//   PCIe Base Spec r2.1, §3.2.1
// ---------------------------------------------------------------------------
module pcie_datalink_init
  import pcie_datalink_pkg::*;
(
    input  logic            clk_i,
    input  logic            rst_i,
    input  logic            phy_link_up_i,
    output logic            init_flow_control_o,
    output logic            soft_reset_o,
    output pcie_dl_status_e link_status_o,
    // ---- flow control initialization ---------------------------------------
    input  logic            fc1_values_stored_i,
    input  logic            fc2_values_stored_i,
    input  logic            init_ack_i
);

  // -------------------------------------------------------------------------
  // State machine
  // -------------------------------------------------------------------------
  // State           Action                        Exit
  // ST_DL_INACTIVE  DL_DOWN, soft_reset_o high    phy_link_up_i: ST_DL_INIT
  // ST_DL_INIT      flow control initialization   init_ack_i: ST_DL_INIT_FC1
  //                 started
  // ST_DL_INIT_FC1  waits for the peer's InitFC   InitFC1 or InitFC2 values
  //                                               stored: ST_DL_INIT_FC2
  // ST_DL_INIT_FC2  DL_UP                         InitFC2 values stored:
  //                                               ST_DL_ACTIVE
  // ST_DL_ACTIVE    DL_ACTIVE                     none but link down
  // Every other state returns to ST_DL_INACTIVE when Physical LinkUp is lost,
  // and soft_reset_o rises with it (PCIe Base Spec r2.1, §3.2.1).
  typedef enum logic [2:0] {
    ST_DL_INACTIVE,
    ST_DL_INIT,
    ST_DL_INIT_FC1,
    ST_DL_INIT_FC2,
    ST_DL_ACTIVE
  } pcie_dl_state_e;

  pcie_dl_state_e        next_state;
  pcie_dl_state_e        curr_state;
  logic            [6:0] link_state;

  pcie_dl_status_e       link_status_c;
  pcie_dl_status_e       link_status_r;
  logic                  init_flow_control_c;
  logic                  init_flow_control_r;
  logic                  soft_reset_r;
  logic                  soft_reset_c;

  // Reset enters ST_DL_INACTIVE with soft_reset_r high, so the rest of the
  // Data Link Layer stays in reset until Physical LinkUp.
  always_ff @(posedge clk_i or posedge rst_i) begin
    if (rst_i) begin
      curr_state          <= ST_DL_INACTIVE;
      link_status_r       <= DL_DOWN;
      init_flow_control_r <= '0;
      soft_reset_r        <= '1;
    end else begin
      curr_state          <= next_state;
      link_status_r       <= link_status_c;
      init_flow_control_r <= init_flow_control_c;
      soft_reset_r        <= soft_reset_c;
    end
  end

  always_comb begin : combo_block
    next_state          = curr_state;
    init_flow_control_c = init_flow_control_r;
    soft_reset_c        = soft_reset_r;
    link_status_c       = link_status_r;
    case (curr_state)
      ST_DL_INACTIVE: begin
        link_status_c = DL_DOWN;
        if (phy_link_up_i) begin
          next_state          = ST_DL_INIT;
          init_flow_control_c = '1;
          soft_reset_c        = '0;
        end
      end
      ST_DL_INIT: begin
        if (!phy_link_up_i) begin
          next_state   = ST_DL_INACTIVE;
          soft_reset_c = '1;
        end else begin
          if (init_ack_i) begin
            next_state = ST_DL_INIT_FC1;
          end
        end
      end
      // DL_Down is reported in FC_INIT1 and DL_Up in FC_INIT2 (PCIe Base Spec
      // r2.1, §3.2.1).
      ST_DL_INIT_FC1: begin
        if (!phy_link_up_i) begin
          next_state   = ST_DL_INACTIVE;
          soft_reset_c = '1;
        end else begin
          if (fc1_values_stored_i || fc2_values_stored_i) begin
            next_state    = ST_DL_INIT_FC2;
            link_status_c = DL_UP;
          end
        end
      end
      ST_DL_INIT_FC2: begin
        if (!phy_link_up_i) begin
          next_state   = ST_DL_INACTIVE;
          soft_reset_c = '1;
        end else begin
          if (fc2_values_stored_i) begin
            next_state    = ST_DL_ACTIVE;
            link_status_c = DL_ACTIVE;
          end
        end
      end
      ST_DL_ACTIVE: begin
        if (!phy_link_up_i) begin
          next_state   = ST_DL_INACTIVE;
          soft_reset_c = '1;
        end
      end
      default: begin
      end
    endcase
  end

  assign init_flow_control_o = init_flow_control_r;
  assign soft_reset_o        = soft_reset_r;
  assign link_status_o       = link_status_r;
endmodule
