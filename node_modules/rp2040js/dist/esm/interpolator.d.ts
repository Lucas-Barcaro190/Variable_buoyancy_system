export declare class InterpolatorConfig {
    shift: number;
    maskLSB: number;
    maskMSB: number;
    signed: boolean;
    crossInput: boolean;
    crossResult: boolean;
    addRaw: boolean;
    forceMSB: number;
    blend: boolean;
    clamp: boolean;
    overf0: boolean;
    overf1: boolean;
    overf: boolean;
    constructor(value: number);
    toUint32(): number;
}
export declare class Interpolator {
    private readonly index;
    accum0: number;
    accum1: number;
    base0: number;
    base1: number;
    base2: number;
    ctrl0: number;
    ctrl1: number;
    result0: number;
    result1: number;
    result2: number;
    smresult0: number;
    smresult1: number;
    constructor(index: number);
    update(): void;
    writeback(): void;
    setBase01(value: number): void;
}
