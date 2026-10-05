// Chuck Benz, Hollis, NH   Copyright (c)2002
//
// The information and description contained herein is the
// property of Chuck Benz.
//
// Permission is granted for any reuse of this information
// and description as long as this copyright notice is
// preserved.  Modifications may be made as long as this
// notice is preserved.

// ---------------------------------------------------------------------------
// encode_8b10b -- combinational 8b/10b encoder for one Symbol
//
// Purpose
//   Encodes a byte and its control bit into a 10-bit Symbol with the 8b/10b
//   transmission code of PCIe Base Spec r2.1, §4.2.1, taking the code-group
//   from the column for the current running disparity. pcie_endpoint_top
//   chains two instances per lane, one per Symbol of the 16-bit PIPE word,
//   with the first one's dispout driving the second one's dispin.
//
// Interfaces
//   Symbol in   datain: {K, H, G, F, E, D, C, B, A}; datain[8] = 1 requests a
//               K code-group (a Special Symbol).
//   Symbol out  dataout: {j, h, g, f, i, e, d, c, b, a}; bit a goes on the
//               Lane first.
//   Disparity   dispin: running disparity before the Symbol; dispout: running
//               disparity after it.
//   Check       illegal_k_o: a K code-group requested for a byte outside the
//               twelve Special Symbols.
//
// Clock and reset
//   None: the module is combinational.
//
// References
//   PCIe Base Spec r2.1, §4.2.1
//   PCIe Base Spec r2.1, §4.2.1.1
//   PCIe Base Spec r2.1, Table B-1
//   PCIe Base Spec r2.1, Table B-2
// ---------------------------------------------------------------------------
module encode_8b10b (
    datain,
    dispin,
    dataout,
    dispout,
    illegal_k_o
);
  input [8:0] datain;
  input dispin;  // 0 = neg disp; 1 = pos disp
  output [9:0] dataout;
  output dispout;
  // High when datain[8] = 1 for a byte with no K code-group in PCIe Base Spec
  // r2.1, Table B-2; valid in the same cycle as datain.
  output illegal_k_o;


  wire ai = datain[0];
  wire bi = datain[1];
  wire ci = datain[2];
  wire di = datain[3];
  wire ei = datain[4];
  wire fi = datain[5];
  wire gi = datain[6];
  wire hi = datain[7];
  wire ki = datain[8];

  wire aeqb = (ai & bi) | (!ai & !bi);
  wire ceqd = (ci & di) | (!ci & !di);
  wire l22 = (ai & bi & !ci & !di) | (ci & di & !ai & !bi) | (!aeqb & !ceqd);
  wire l40 = ai & bi & ci & di;
  wire l04 = !ai & !bi & !ci & !di;
  wire l13 = (!aeqb & !ci & !di) | (!ceqd & !ai & !bi);
  wire l31 = (!aeqb & ci & di) | (!ceqd & ai & bi);

  // The 5B/6B encoding

  wire ao = ai;
  wire bo = (bi & !l40) | l04;
  wire co = l04 | ci | (ei & di & !ci & !bi & !ai);
  wire do_ = di & !(ai & bi & ci);
  wire eo = (ei | l13) & !(ei & di & !ci & !bi & !ai);
  wire io = (l22 & !ei) | (ei & !di & !ci & !(ai & bi)) |  // D16, D17, D18
  (ei & l40) | (ki & ei & di & ci & !bi & !ai) |  // K.28
  (ei & !di & ci & !bi & !ai);

  // pds16 indicates cases where d-1 is assumed + to get our encoded value
  wire pd1s6 = (ei & di & !ci & !bi & !ai) | (!ei & !l22 & !l31);
  // nds16 indicates cases where d-1 is assumed - to get our encoded value
  wire nd1s6 = ki | (ei & !l22 & !l13) | (!ei & !di & ci & bi & ai);

  // ndos6 is pds16 cases where d-1 is + yields - disp out - all of them
  wire ndos6 = pd1s6;
  // pdos6 is nds16 cases where d-1 is - yields + disp out - all but one
  wire pdos6 = ki | (ei & !l22 & !l13);


  // The normal coding Dx.P7 would give a run length of 5 for D17, D18 and D20
  // at negative running disparity and for D11, D13 and D14 at positive, so
  // these take the alternate coding Dx.A7. Every Kx.7 takes the A7 fghj
  // (PCIe Base Spec r2.1, Table B-2).
  wire alt7 = fi & gi & hi & (ki | (dispin ? (!ei & di & l31) : (ei & !di & l13)));


  wire fo = fi & !alt7;
  wire go = gi | (!fi & !gi & !hi);
  wire ho = hi;
  wire jo = (!hi & (gi ^ fi)) | alt7;

  // nd1s4 is cases where d-1 is assumed - to get our encoded value
  wire nd1s4 = fi & gi;
  // pd1s4 is cases where d-1 is assumed + to get our encoded value
  wire pd1s4 = (!fi & !gi) | (ki & ((fi & !gi) | (!fi & gi)));

  // ndos4 is pd1s4 cases where d-1 is + yields - disp out - just some
  wire ndos4 = (!fi & !gi);
  // pdos4 is nd1s4 cases where d-1 is - yields + disp out
  wire pdos4 = fi & gi & hi;

  // only legal K codes are K28.0->.7, K23/27/29/30.7
  //	K28.0->7 is ei=di=ci=1,bi=ai=0
  //	K23 is 10111
  //	K27 is 11011
  //	K29 is 11101
  //	K30 is 11110 - so K23/27/29/30 are ei & l31
  wire illegalk = ki & (ai | bi | !ci | !di | !ei) &  // not K28.0->7
  (!fi | !gi | !hi | !ei | !l31);  // not K23/27/29/30.7

  assign illegal_k_o = illegalk;

  // now determine whether to do the complementing
  // complement if prev disp is - and pd1s6 is set, or + and nd1s6 is set
  wire compls6 = (pd1s6 & !dispin) | (nd1s6 & dispin);

  // disparity out of 5b6b is disp in with pdso6 and ndso6
  // pds16 indicates cases where d-1 is assumed + to get our encoded value
  // ndos6 is cases where d-1 is + yields - disp out
  // nds16 indicates cases where d-1 is assumed - to get our encoded value
  // pdos6 is cases where d-1 is - yields + disp out
  // disp toggles in all ndis16 cases, and all but that 1 nds16 case

  wire disp6 = dispin ^ (ndos6 | pdos6);

  wire compls4 = (pd1s4 & !disp6) | (nd1s4 & disp6);
  assign dispout = disp6 ^ (ndos4 | pdos4);

  assign dataout = {
    (jo ^ compls4),
    (ho ^ compls4),
    (go ^ compls4),
    (fo ^ compls4),
    (io ^ compls6),
    (eo ^ compls6),
    (do_ ^ compls6),
    (co ^ compls6),
    (bo ^ compls6),
    (ao ^ compls6)
  };

endmodule
