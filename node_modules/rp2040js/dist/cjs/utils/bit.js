"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.bit = bit;
exports.s32 = s32;
exports.u32 = u32;
function bit(n) {
    return 1 << n;
}
function s32(n) {
    return n | 0;
}
function u32(n) {
    return n >>> 0;
}
