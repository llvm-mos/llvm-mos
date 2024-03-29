# RUN: llvm-mc -triple x86_64 -show-encoding %s | FileCheck %s
# RUN: not llvm-mc -triple i386 -show-encoding %s 2>&1 | FileCheck %s --check-prefix=ERROR

# ERROR-COUNT-47: error:
# ERROR-NOT: error:
# CHECK: {evex}	rclb	$123, %bl
# CHECK: encoding: [0x62,0xf4,0x7c,0x08,0xc0,0xd3,0x7b]
         {evex}	rclb	$123, %bl
# CHECK: rclb	$123, %bl, %bl
# CHECK: encoding: [0x62,0xf4,0x64,0x18,0xc0,0xd3,0x7b]
         rclb	$123, %bl, %bl
# CHECK: {evex}	rclw	$123, %dx
# CHECK: encoding: [0x62,0xf4,0x7d,0x08,0xc1,0xd2,0x7b]
         {evex}	rclw	$123, %dx
# CHECK: rclw	$123, %dx, %dx
# CHECK: encoding: [0x62,0xf4,0x6d,0x18,0xc1,0xd2,0x7b]
         rclw	$123, %dx, %dx
# CHECK: {evex}	rcll	$123, %ecx
# CHECK: encoding: [0x62,0xf4,0x7c,0x08,0xc1,0xd1,0x7b]
         {evex}	rcll	$123, %ecx
# CHECK: rcll	$123, %ecx, %ecx
# CHECK: encoding: [0x62,0xf4,0x74,0x18,0xc1,0xd1,0x7b]
         rcll	$123, %ecx, %ecx
# CHECK: {evex}	rclq	$123, %r9
# CHECK: encoding: [0x62,0xd4,0xfc,0x08,0xc1,0xd1,0x7b]
         {evex}	rclq	$123, %r9
# CHECK: rclq	$123, %r9, %r9
# CHECK: encoding: [0x62,0xd4,0xb4,0x18,0xc1,0xd1,0x7b]
         rclq	$123, %r9, %r9
# CHECK: {evex}	rclb	$123, 291(%r8,%rax,4)
# CHECK: encoding: [0x62,0xd4,0x7c,0x08,0xc0,0x94,0x80,0x23,0x01,0x00,0x00,0x7b]
         {evex}	rclb	$123, 291(%r8,%rax,4)
# CHECK: rclb	$123, 291(%r8,%rax,4), %bl
# CHECK: encoding: [0x62,0xd4,0x64,0x18,0xc0,0x94,0x80,0x23,0x01,0x00,0x00,0x7b]
         rclb	$123, 291(%r8,%rax,4), %bl
# CHECK: {evex}	rclw	$123, 291(%r8,%rax,4)
# CHECK: encoding: [0x62,0xd4,0x7d,0x08,0xc1,0x94,0x80,0x23,0x01,0x00,0x00,0x7b]
         {evex}	rclw	$123, 291(%r8,%rax,4)
# CHECK: rclw	$123, 291(%r8,%rax,4), %dx
# CHECK: encoding: [0x62,0xd4,0x6d,0x18,0xc1,0x94,0x80,0x23,0x01,0x00,0x00,0x7b]
         rclw	$123, 291(%r8,%rax,4), %dx
# CHECK: {evex}	rcll	$123, 291(%r8,%rax,4)
# CHECK: encoding: [0x62,0xd4,0x7c,0x08,0xc1,0x94,0x80,0x23,0x01,0x00,0x00,0x7b]
         {evex}	rcll	$123, 291(%r8,%rax,4)
# CHECK: rcll	$123, 291(%r8,%rax,4), %ecx
# CHECK: encoding: [0x62,0xd4,0x74,0x18,0xc1,0x94,0x80,0x23,0x01,0x00,0x00,0x7b]
         rcll	$123, 291(%r8,%rax,4), %ecx
# CHECK: {evex}	rclq	$123, 291(%r8,%rax,4)
# CHECK: encoding: [0x62,0xd4,0xfc,0x08,0xc1,0x94,0x80,0x23,0x01,0x00,0x00,0x7b]
         {evex}	rclq	$123, 291(%r8,%rax,4)
# CHECK: rclq	$123, 291(%r8,%rax,4), %r9
# CHECK: encoding: [0x62,0xd4,0xb4,0x18,0xc1,0x94,0x80,0x23,0x01,0x00,0x00,0x7b]
         rclq	$123, 291(%r8,%rax,4), %r9
# CHECK: {evex}	rclb	%bl
# CHECK: encoding: [0x62,0xf4,0x7c,0x08,0xd0,0xd3]
         {evex}	rclb	%bl
# CHECK: {evex}	rclb	%cl, %bl
# CHECK: encoding: [0x62,0xf4,0x7c,0x08,0xd2,0xd3]
         {evex}	rclb	%cl, %bl
# CHECK: rclb	%cl, %bl, %bl
# CHECK: encoding: [0x62,0xf4,0x64,0x18,0xd2,0xd3]
         rclb	%cl, %bl, %bl
# CHECK: {evex}	rclw	%cl, %dx
# CHECK: encoding: [0x62,0xf4,0x7d,0x08,0xd3,0xd2]
         {evex}	rclw	%cl, %dx
# CHECK: rclw	%cl, %dx, %dx
# CHECK: encoding: [0x62,0xf4,0x6d,0x18,0xd3,0xd2]
         rclw	%cl, %dx, %dx
# CHECK: {evex}	rcll	%cl, %ecx
# CHECK: encoding: [0x62,0xf4,0x7c,0x08,0xd3,0xd1]
         {evex}	rcll	%cl, %ecx
# CHECK: rcll	%cl, %ecx, %ecx
# CHECK: encoding: [0x62,0xf4,0x74,0x18,0xd3,0xd1]
         rcll	%cl, %ecx, %ecx
# CHECK: {evex}	rclq	%cl, %r9
# CHECK: encoding: [0x62,0xd4,0xfc,0x08,0xd3,0xd1]
         {evex}	rclq	%cl, %r9
# CHECK: rclq	%cl, %r9, %r9
# CHECK: encoding: [0x62,0xd4,0xb4,0x18,0xd3,0xd1]
         rclq	%cl, %r9, %r9
# CHECK: {evex}	rclb	%cl, 291(%r8,%rax,4)
# CHECK: encoding: [0x62,0xd4,0x7c,0x08,0xd2,0x94,0x80,0x23,0x01,0x00,0x00]
         {evex}	rclb	%cl, 291(%r8,%rax,4)
# CHECK: rclb	%cl, 291(%r8,%rax,4), %bl
# CHECK: encoding: [0x62,0xd4,0x64,0x18,0xd2,0x94,0x80,0x23,0x01,0x00,0x00]
         rclb	%cl, 291(%r8,%rax,4), %bl
# CHECK: {evex}	rclw	%cl, 291(%r8,%rax,4)
# CHECK: encoding: [0x62,0xd4,0x7d,0x08,0xd3,0x94,0x80,0x23,0x01,0x00,0x00]
         {evex}	rclw	%cl, 291(%r8,%rax,4)
# CHECK: rclw	%cl, 291(%r8,%rax,4), %dx
# CHECK: encoding: [0x62,0xd4,0x6d,0x18,0xd3,0x94,0x80,0x23,0x01,0x00,0x00]
         rclw	%cl, 291(%r8,%rax,4), %dx
# CHECK: {evex}	rcll	%cl, 291(%r8,%rax,4)
# CHECK: encoding: [0x62,0xd4,0x7c,0x08,0xd3,0x94,0x80,0x23,0x01,0x00,0x00]
         {evex}	rcll	%cl, 291(%r8,%rax,4)
# CHECK: rcll	%cl, 291(%r8,%rax,4), %ecx
# CHECK: encoding: [0x62,0xd4,0x74,0x18,0xd3,0x94,0x80,0x23,0x01,0x00,0x00]
         rcll	%cl, 291(%r8,%rax,4), %ecx
# CHECK: {evex}	rclq	%cl, 291(%r8,%rax,4)
# CHECK: encoding: [0x62,0xd4,0xfc,0x08,0xd3,0x94,0x80,0x23,0x01,0x00,0x00]
         {evex}	rclq	%cl, 291(%r8,%rax,4)
# CHECK: rclq	%cl, 291(%r8,%rax,4), %r9
# CHECK: encoding: [0x62,0xd4,0xb4,0x18,0xd3,0x94,0x80,0x23,0x01,0x00,0x00]
         rclq	%cl, 291(%r8,%rax,4), %r9
# CHECK: {evex}	rclw	%dx
# CHECK: encoding: [0x62,0xf4,0x7d,0x08,0xd1,0xd2]
         {evex}	rclw	%dx
# CHECK: rclw	%dx, %dx
# CHECK: encoding: [0x62,0xf4,0x6d,0x18,0xd1,0xd2]
         rclw	%dx, %dx
# CHECK: {evex}	rcll	%ecx
# CHECK: encoding: [0x62,0xf4,0x7c,0x08,0xd1,0xd1]
         {evex}	rcll	%ecx
# CHECK: rcll	%ecx, %ecx
# CHECK: encoding: [0x62,0xf4,0x74,0x18,0xd1,0xd1]
         rcll	%ecx, %ecx
# CHECK: {evex}	rclq	%r9
# CHECK: encoding: [0x62,0xd4,0xfc,0x08,0xd1,0xd1]
         {evex}	rclq	%r9
# CHECK: rclq	%r9, %r9
# CHECK: encoding: [0x62,0xd4,0xb4,0x18,0xd1,0xd1]
         rclq	%r9, %r9
# CHECK: {evex}	rclb	291(%r8,%rax,4)
# CHECK: encoding: [0x62,0xd4,0x7c,0x08,0xd0,0x94,0x80,0x23,0x01,0x00,0x00]
         {evex}	rclb	291(%r8,%rax,4)
# CHECK: rclb	291(%r8,%rax,4), %bl
# CHECK: encoding: [0x62,0xd4,0x64,0x18,0xd0,0x94,0x80,0x23,0x01,0x00,0x00]
         rclb	291(%r8,%rax,4), %bl
# CHECK: {evex}	rclw	291(%r8,%rax,4)
# CHECK: encoding: [0x62,0xd4,0x7d,0x08,0xd1,0x94,0x80,0x23,0x01,0x00,0x00]
         {evex}	rclw	291(%r8,%rax,4)
# CHECK: rclw	291(%r8,%rax,4), %dx
# CHECK: encoding: [0x62,0xd4,0x6d,0x18,0xd1,0x94,0x80,0x23,0x01,0x00,0x00]
         rclw	291(%r8,%rax,4), %dx
# CHECK: {evex}	rcll	291(%r8,%rax,4)
# CHECK: encoding: [0x62,0xd4,0x7c,0x08,0xd1,0x94,0x80,0x23,0x01,0x00,0x00]
         {evex}	rcll	291(%r8,%rax,4)
# CHECK: rcll	291(%r8,%rax,4), %ecx
# CHECK: encoding: [0x62,0xd4,0x74,0x18,0xd1,0x94,0x80,0x23,0x01,0x00,0x00]
         rcll	291(%r8,%rax,4), %ecx
# CHECK: {evex}	rclq	291(%r8,%rax,4)
# CHECK: encoding: [0x62,0xd4,0xfc,0x08,0xd1,0x94,0x80,0x23,0x01,0x00,0x00]
         {evex}	rclq	291(%r8,%rax,4)
# CHECK: rclq	291(%r8,%rax,4), %r9
# CHECK: encoding: [0x62,0xd4,0xb4,0x18,0xd1,0x94,0x80,0x23,0x01,0x00,0x00]
         rclq	291(%r8,%rax,4), %r9
