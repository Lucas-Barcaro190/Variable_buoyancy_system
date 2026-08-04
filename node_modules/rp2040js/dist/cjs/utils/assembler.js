"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.opcodeADCS = opcodeADCS;
exports.opcodeADDS1 = opcodeADDS1;
exports.opcodeADDS2 = opcodeADDS2;
exports.opcodeADDspPlusImm = opcodeADDspPlusImm;
exports.opcodeADDsp2 = opcodeADDsp2;
exports.opcodeADDSreg = opcodeADDSreg;
exports.opcodeADDreg = opcodeADDreg;
exports.opcodeADR = opcodeADR;
exports.opcodeANDS = opcodeANDS;
exports.opcodeASRS = opcodeASRS;
exports.opcodeASRSreg = opcodeASRSreg;
exports.opcodeBT1 = opcodeBT1;
exports.opcodeBT2 = opcodeBT2;
exports.opcodeBICS = opcodeBICS;
exports.opcodeBL = opcodeBL;
exports.opcodeBLX = opcodeBLX;
exports.opcodeBX = opcodeBX;
exports.opcodeCMN = opcodeCMN;
exports.opcodeCMPimm = opcodeCMPimm;
exports.opcodeCMPregT1 = opcodeCMPregT1;
exports.opcodeCMPregT2 = opcodeCMPregT2;
exports.opcodeDMBSY = opcodeDMBSY;
exports.opcodeDSBSY = opcodeDSBSY;
exports.opcodeEORS = opcodeEORS;
exports.opcodeISBSY = opcodeISBSY;
exports.opcodeLDMIA = opcodeLDMIA;
exports.opcodeLDRreg = opcodeLDRreg;
exports.opcodeLDRimm = opcodeLDRimm;
exports.opcodeLDRlit = opcodeLDRlit;
exports.opcodeLDRB = opcodeLDRB;
exports.opcodeLDRsp = opcodeLDRsp;
exports.opcodeLDRBreg = opcodeLDRBreg;
exports.opcodeLDRH = opcodeLDRH;
exports.opcodeLDRHreg = opcodeLDRHreg;
exports.opcodeLDRSB = opcodeLDRSB;
exports.opcodeLDRSH = opcodeLDRSH;
exports.opcodeLSLSreg = opcodeLSLSreg;
exports.opcodeLSLSimm = opcodeLSLSimm;
exports.opcodeLSRS = opcodeLSRS;
exports.opcodeLSRSreg = opcodeLSRSreg;
exports.opcodeMOV = opcodeMOV;
exports.opcodeMOVS = opcodeMOVS;
exports.opcodeMOVSreg = opcodeMOVSreg;
exports.opcodeMRS = opcodeMRS;
exports.opcodeMSR = opcodeMSR;
exports.opcodeMULS = opcodeMULS;
exports.opcodeMVNS = opcodeMVNS;
exports.opcodeNOP = opcodeNOP;
exports.opcodeORRS = opcodeORRS;
exports.opcodePOP = opcodePOP;
exports.opcodePUSH = opcodePUSH;
exports.opcodeREV = opcodeREV;
exports.opcodeREV16 = opcodeREV16;
exports.opcodeREVSH = opcodeREVSH;
exports.opcodeROR = opcodeROR;
exports.opcodeRSBS = opcodeRSBS;
exports.opcodeSBCS = opcodeSBCS;
exports.opcodeSTMIA = opcodeSTMIA;
exports.opcodeSTR = opcodeSTR;
exports.opcodeSTRsp = opcodeSTRsp;
exports.opcodeSTRreg = opcodeSTRreg;
exports.opcodeSTRB = opcodeSTRB;
exports.opcodeSTRBreg = opcodeSTRBreg;
exports.opcodeSTRH = opcodeSTRH;
exports.opcodeSTRHreg = opcodeSTRHreg;
exports.opcodeSUBS1 = opcodeSUBS1;
exports.opcodeSUBS2 = opcodeSUBS2;
exports.opcodeSUBSreg = opcodeSUBSreg;
exports.opcodeSUBsp = opcodeSUBsp;
exports.opcodeSVC = opcodeSVC;
exports.opcodeSXTB = opcodeSXTB;
exports.opcodeSXTH = opcodeSXTH;
exports.opcodeTST = opcodeTST;
exports.opcodeUXTB = opcodeUXTB;
exports.opcodeUDF = opcodeUDF;
exports.opcodeUDF2 = opcodeUDF2;
exports.opcodeUXTH = opcodeUXTH;
exports.opcodeWFI = opcodeWFI;
exports.opcodeYIELD = opcodeYIELD;
function opcodeADCS(Rdn, Rm) {
    return (0b0100000101 << 6) | ((Rm & 7) << 3) | (Rdn & 7);
}
function opcodeADDS1(Rd, Rn, imm3) {
    return (0b0001110 << 9) | ((imm3 & 0x7) << 6) | ((Rn & 7) << 3) | (Rd & 7);
}
function opcodeADDS2(Rdn, imm8) {
    return (0b00110 << 11) | ((Rdn & 7) << 8) | (imm8 & 0xff);
}
function opcodeADDspPlusImm(Rd, imm8) {
    return (0b10101 << 11) | ((Rd & 7) << 8) | ((imm8 >> 2) & 0xff);
}
function opcodeADDsp2(imm) {
    return (0b101100000 << 7) | ((imm >> 2) & 0x7f);
}
function opcodeADDSreg(Rd, Rn, Rm) {
    return (0b0001100 << 9) | ((Rm & 0x7) << 6) | ((Rn & 7) << 3) | (Rd & 7);
}
function opcodeADDreg(Rdn, Rm) {
    return (0b01000100 << 8) | ((Rdn & 0x8) << 4) | ((Rm & 0xf) << 3) | (Rdn & 0x7);
}
function opcodeADR(Rd, imm8) {
    return (0b10100 << 11) | ((Rd & 7) << 8) | ((imm8 >> 2) & 0xff);
}
function opcodeANDS(Rn, Rm) {
    return (0b0100000000 << 6) | ((Rm & 7) << 3) | (Rn & 0x7);
}
function opcodeASRS(Rd, Rm, imm5) {
    return (0b00010 << 11) | ((imm5 & 0x1f) << 6) | ((Rm & 0x7) << 3) | (Rd & 0x7);
}
function opcodeASRSreg(Rdn, Rm) {
    return (0b0100000100 << 6) | ((Rm & 0x7) << 3) | ((Rm & 0x7) << 3) | (Rdn & 0x7);
}
function opcodeBT1(cond, imm8) {
    return (0b1101 << 12) | ((cond & 0xf) << 8) | ((imm8 >> 1) & 0x1ff);
}
function opcodeBT2(imm11) {
    return (0b11100 << 11) | ((imm11 >> 1) & 0x7ff);
}
function opcodeBICS(Rdn, Rm) {
    return (0b0100001110 << 6) | ((Rm & 7) << 3) | (Rdn & 7);
}
function opcodeBL(imm) {
    const imm11 = (imm >> 1) & 0x7ff;
    const imm10 = (imm >> 12) & 0x3ff;
    const s = imm < 0 ? 1 : 0;
    const j2 = 1 - (((imm >> 22) & 0x1) ^ s);
    const j1 = 1 - (((imm >> 23) & 0x1) ^ s);
    const opcode = (0b1101 << 28) | (j1 << 29) | (j2 << 27) | (imm11 << 16) | (0b11110 << 11) | (s << 10) | imm10;
    return opcode >>> 0;
}
function opcodeBLX(Rm) {
    return (0b010001111 << 7) | (Rm << 3);
}
function opcodeBX(Rm) {
    return (0b010001110 << 7) | (Rm << 3);
}
function opcodeCMN(Rn, Rm) {
    return (0b0100001011 << 6) | ((Rm & 0x7) << 3) | (Rn & 0x7);
}
function opcodeCMPimm(Rn, Imm8) {
    return (0b00101 << 11) | ((Rn & 0x7) << 8) | (Imm8 & 0xff);
}
function opcodeCMPregT1(Rn, Rm) {
    return (0b0100001010 << 6) | ((Rm & 0x7) << 3) | (Rn & 0x7);
}
function opcodeCMPregT2(Rn, Rm) {
    return (0b01000101 << 8) | (((Rn >> 3) & 0x1) << 7) | ((Rm & 0xf) << 3) | (Rn & 0x7);
}
function opcodeDMBSY() {
    return 0x8f50f3bf;
}
function opcodeDSBSY() {
    return 0x8f4ff3bf;
}
function opcodeEORS(Rdn, Rm) {
    return (0b0100000001 << 6) | ((Rm & 0x7) << 3) | (Rdn & 0x7);
}
function opcodeISBSY() {
    return 0x8f6ff3bf;
}
function opcodeLDMIA(Rn, registers) {
    return (0b11001 << 11) | ((Rn & 0x7) << 8) | (registers & 0xff);
}
function opcodeLDRreg(Rt, Rn, Rm) {
    return (0b0101100 << 9) | ((Rm & 0x7) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}
function opcodeLDRimm(Rt, Rn, imm5) {
    return (0b01101 << 11) | (((imm5 >> 2) & 0x1f) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}
function opcodeLDRlit(Rt, imm8) {
    return (0b01001 << 11) | ((imm8 >> 2) & 0xff) | ((Rt & 0x7) << 8);
}
function opcodeLDRB(Rt, Rn, imm5) {
    return (0b01111 << 11) | ((imm5 & 0x1f) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}
function opcodeLDRsp(Rt, imm8) {
    return (0b10011 << 11) | ((Rt & 7) << 8) | ((imm8 >> 2) & 0xff);
}
function opcodeLDRBreg(Rt, Rn, Rm) {
    return (0b0101110 << 9) | ((Rm & 0x7) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}
function opcodeLDRH(Rt, Rn, imm5) {
    return (0b10001 << 11) | (((imm5 >> 1) & 0xf) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}
function opcodeLDRHreg(Rt, Rn, Rm) {
    return (0b0101101 << 9) | ((Rm & 0x7) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}
function opcodeLDRSB(Rt, Rn, Rm) {
    return (0b0101011 << 9) | ((Rm & 0x7) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}
function opcodeLDRSH(Rt, Rn, Rm) {
    return (0b0101111 << 9) | ((Rm & 0x7) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}
function opcodeLSLSreg(Rdn, Rm) {
    return (0b0100000010 << 6) | ((Rm & 0x7) << 3) | (Rdn & 0x7);
}
function opcodeLSLSimm(Rd, Rm, Imm5) {
    return (0b00000 << 11) | ((Imm5 & 0x1f) << 6) | ((Rm & 0x7) << 3) | (Rd & 0x7);
}
function opcodeLSRS(Rd, Rm, imm5) {
    return (0b00001 << 11) | ((imm5 & 0x1f) << 6) | ((Rm & 0x7) << 3) | (Rd & 0x7);
}
function opcodeLSRSreg(Rdn, Rm) {
    return (0b0100000011 << 6) | ((Rm & 0x7) << 3) | (Rdn & 0x7);
}
function opcodeMOV(Rd, Rm) {
    return (0b01000110 << 8) | ((Rd & 0x8 ? 1 : 0) << 7) | (Rm << 3) | (Rd & 0x7);
}
function opcodeMOVS(Rd, imm8) {
    return (0b00100 << 11) | ((Rd & 0x7) << 8) | (imm8 & 0xff);
}
function opcodeMOVSreg(Rd, Rm) {
    return (0b000000000 << 6) | ((Rm & 0x7) << 3) | (Rd & 0x7);
}
function opcodeMRS(Rd, specReg) {
    return (((0b1000 << 28) | ((Rd & 0xf) << 24) | ((specReg & 0xff) << 16) | 0b1111001111101111) >>> 0);
}
function opcodeMSR(specReg, Rn) {
    return ((0b10001000 << 24) | ((specReg & 0xff) << 16) | (0b111100111000 << 4) | (Rn & 0xf)) >>> 0;
}
function opcodeMULS(Rn, Rdm) {
    return (0b0100001101 << 6) | ((Rn & 7) << 3) | (Rdm & 7);
}
function opcodeMVNS(Rd, Rm) {
    return (0b0100001111 << 6) | ((Rm & 7) << 3) | (Rd & 7);
}
function opcodeNOP() {
    return 0b1011111100000000;
}
function opcodeORRS(Rn, Rm) {
    return (0b0100001100 << 6) | ((Rm & 0x7) << 3) | (Rn & 0x7);
}
function opcodePOP(P, registerList) {
    return (0b1011110 << 9) | ((P ? 1 : 0) << 8) | registerList;
}
function opcodePUSH(M, registerList) {
    return (0b1011010 << 9) | ((M ? 1 : 0) << 8) | registerList;
}
function opcodeREV(Rd, Rn) {
    return (0b1011101000 << 6) | ((Rn & 0x7) << 3) | (Rd & 0x7);
}
function opcodeREV16(Rd, Rn) {
    return (0b1011101001 << 6) | ((Rn & 0x7) << 3) | (Rd & 0x7);
}
function opcodeREVSH(Rd, Rn) {
    return (0b1011101011 << 6) | ((Rn & 0x7) << 3) | (Rd & 0x7);
}
function opcodeROR(Rdn, Rm) {
    return (0b0100000111 << 6) | ((Rm & 0x7) << 3) | (Rdn & 0x7);
}
function opcodeRSBS(Rd, Rn) {
    return (0b0100001001 << 6) | ((Rn & 0x7) << 3) | (Rd & 0x7);
}
function opcodeSBCS(Rn, Rm) {
    return (0b0100000110 << 6) | ((Rm & 0x7) << 3) | (Rn & 0x7);
}
function opcodeSTMIA(Rn, registers) {
    return (0b11000 << 11) | ((Rn & 0x7) << 8) | (registers & 0xff);
}
function opcodeSTR(Rt, Rm, imm5) {
    return (0b01100 << 11) | (((imm5 >> 2) & 0x1f) << 6) | ((Rm & 0x7) << 3) | (Rt & 0x7);
}
function opcodeSTRsp(Rt, imm8) {
    return (0b10010 << 11) | ((Rt & 7) << 8) | ((imm8 >> 2) & 0xff);
}
function opcodeSTRreg(Rt, Rn, Rm) {
    return (0b0101000 << 9) | ((Rm & 0x7) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}
function opcodeSTRB(Rt, Rm, imm5) {
    return (0b01110 << 11) | ((imm5 & 0x1f) << 6) | ((Rm & 0x7) << 3) | (Rt & 0x7);
}
function opcodeSTRBreg(Rt, Rn, Rm) {
    return (0b0101010 << 9) | ((Rm & 0x7) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}
function opcodeSTRH(Rt, Rm, imm5) {
    return (0b10000 << 11) | (((imm5 >> 1) & 0x1f) << 6) | ((Rm & 0x7) << 3) | (Rt & 0x7);
}
function opcodeSTRHreg(Rt, Rn, Rm) {
    return (0b0101001 << 9) | ((Rm & 0x7) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}
function opcodeSUBS1(Rd, Rn, imm3) {
    return (0b0001111 << 9) | ((imm3 & 0x7) << 6) | ((Rn & 7) << 3) | (Rd & 7);
}
function opcodeSUBS2(Rdn, imm8) {
    return (0b00111 << 11) | ((Rdn & 7) << 8) | (imm8 & 0xff);
}
function opcodeSUBSreg(Rd, Rn, Rm) {
    return (0b0001101 << 9) | ((Rm & 0x7) << 6) | ((Rn & 7) << 3) | (Rd & 7);
}
function opcodeSUBsp(imm) {
    return (0b101100001 << 7) | ((imm >> 2) & 0x7f);
}
function opcodeSVC(imm8) {
    return (0b11011111 << 8) | (imm8 & 0xff);
}
function opcodeSXTB(Rd, Rm) {
    return (0b1011001001 << 6) | ((Rm & 7) << 3) | (Rd & 7);
}
function opcodeSXTH(Rd, Rm) {
    return (0b1011001000 << 6) | ((Rm & 7) << 3) | (Rd & 7);
}
function opcodeTST(Rm, Rn) {
    return (0b0100001000 << 6) | ((Rn & 7) << 3) | (Rm & 7);
}
function opcodeUXTB(Rd, Rm) {
    return (0b1011001011 << 6) | ((Rm & 7) << 3) | (Rd & 7);
}
function opcodeUDF(imm8) {
    return ((0b11011110 << 8) | (imm8 & 0xff)) >>> 0;
}
function opcodeUDF2(imm16) {
    const imm12 = imm16 & 0xfff;
    const imm4 = (imm16 >> 12) & 0xf;
    return ((0b111101111111 << 4) | imm4 | (0b1010 << 28) | (imm12 << 16)) >>> 0;
}
function opcodeUXTH(Rd, Rm) {
    return (0b1011001010 << 6) | ((Rm & 7) << 3) | (Rd & 7);
}
function opcodeWFI() {
    return 0b1011111100110000;
}
function opcodeYIELD() {
    return 0b1011111100010000;
}
