/*
 * nrcosasm.s
 *
 *  Created on: Sep 17, 2025
 *      Author: rafae
 */
 /*
 * nrcosasm.s
 *
 *  Created on: Sep 17, 2025
 *      Author: rafae
 */
.syntax unified
.cpu cortex-m4
.fpu fpv4-sp-d16
.thumb

.global newton_cos_asm
.extern arm_cos_f32
.extern arm_sin_f32

// float newton_cos_asm(float x0, float omega, float phi, int max_iter)
// ABI (hard-float):
//   x0     -> s0
//   omega  -> s1
//   phi    -> s2
//   max_it -> r0
// return  -> s0
newton_cos_asm:
    push {r4, lr}

    mov r4, r0             // r4 = max_iter
    vmov s9, s0            // keep x in s9
    // s1 = omega, s2 = phi already set

1:  cmp r4, #0
    beq 2f

    // t = omega * x + phi
    vmul.f32 s3, s1, s9
    vadd.f32 s3, s3, s2

    // cos_val = arm_cos_f32(t)
    vmov s0, s3            // arg in s0
    bl arm_cos_f32
    vmov s4, s0            // s4 = cos_val

    // f = x*x - cos_val
    vmul.f32 s5, s9, s9
    vsub.f32 s5, s5, s4    // f

    // sin_val = arm_sin_f32(t)
    vmov s0, s3
    bl arm_sin_f32
    vmov s6, s0            // s6 = sin_val

    // df = 2*x + omega*sin_val
    vadd.f32 s7, s9, s9    // 2*x
    vmul.f32 s8, s1, s6    // omega*sin_val
    vadd.f32 s7, s7, s8    // df

    // x = x - f/df
    vdiv.f32 s8, s5, s7
    vsub.f32 s9, s9, s8

    subs r4, r4, #1
    bne 1b

2:  // return x in s0
    vmov s0, s9
    pop {r4, pc}
