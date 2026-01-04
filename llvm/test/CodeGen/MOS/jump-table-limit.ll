; RUN: llc -verify-machineinstrs -disable-block-placement < %s | FileCheck %s

; Test that MOS correctly compiles a 256-entry switch statement.
; This is the maximum size for MOS jump tables (8-bit index register).
; Regression test for the fix that changed the limit from < 256 to <= 256.

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

; Test with 256 entries (Range=256) - exactly at the limit
; Verify the jump table structure and spot-check some entries
define i16 @switch_256_entries(i16 %x) {
; CHECK-LABEL: switch_256_entries:
;
; Verify jump table exists with split low/high byte arrays
; CHECK: .LJTI0_0:
; The table should have 256 low byte entries followed by 256 high byte entries
; Spot-check first entry (case 0)
; CHECK: .byte {{.*}}@mos16lo
; ... (255 more low bytes) ...
; After 256 low bytes, high bytes start
; CHECK: .byte {{.*}}@mos16hi
entry:
  switch i16 %x, label %default [
    i16 0, label %case0
    i16 1, label %case1
    i16 2, label %case2
    i16 3, label %case3
    i16 4, label %case4
    i16 5, label %case5
    i16 6, label %case6
    i16 7, label %case7
    i16 8, label %case8
    i16 9, label %case9
    i16 10, label %case10
    i16 11, label %case11
    i16 12, label %case12
    i16 13, label %case13
    i16 14, label %case14
    i16 15, label %case15
    i16 16, label %case16
    i16 17, label %case17
    i16 18, label %case18
    i16 19, label %case19
    i16 20, label %case20
    i16 21, label %case21
    i16 22, label %case22
    i16 23, label %case23
    i16 24, label %case24
    i16 25, label %case25
    i16 26, label %case26
    i16 27, label %case27
    i16 28, label %case28
    i16 29, label %case29
    i16 30, label %case30
    i16 31, label %case31
    i16 32, label %case32
    i16 33, label %case33
    i16 34, label %case34
    i16 35, label %case35
    i16 36, label %case36
    i16 37, label %case37
    i16 38, label %case38
    i16 39, label %case39
    i16 40, label %case40
    i16 41, label %case41
    i16 42, label %case42
    i16 43, label %case43
    i16 44, label %case44
    i16 45, label %case45
    i16 46, label %case46
    i16 47, label %case47
    i16 48, label %case48
    i16 49, label %case49
    i16 50, label %case50
    i16 51, label %case51
    i16 52, label %case52
    i16 53, label %case53
    i16 54, label %case54
    i16 55, label %case55
    i16 56, label %case56
    i16 57, label %case57
    i16 58, label %case58
    i16 59, label %case59
    i16 60, label %case60
    i16 61, label %case61
    i16 62, label %case62
    i16 63, label %case63
    i16 64, label %case64
    i16 65, label %case65
    i16 66, label %case66
    i16 67, label %case67
    i16 68, label %case68
    i16 69, label %case69
    i16 70, label %case70
    i16 71, label %case71
    i16 72, label %case72
    i16 73, label %case73
    i16 74, label %case74
    i16 75, label %case75
    i16 76, label %case76
    i16 77, label %case77
    i16 78, label %case78
    i16 79, label %case79
    i16 80, label %case80
    i16 81, label %case81
    i16 82, label %case82
    i16 83, label %case83
    i16 84, label %case84
    i16 85, label %case85
    i16 86, label %case86
    i16 87, label %case87
    i16 88, label %case88
    i16 89, label %case89
    i16 90, label %case90
    i16 91, label %case91
    i16 92, label %case92
    i16 93, label %case93
    i16 94, label %case94
    i16 95, label %case95
    i16 96, label %case96
    i16 97, label %case97
    i16 98, label %case98
    i16 99, label %case99
    i16 100, label %case100
    i16 101, label %case101
    i16 102, label %case102
    i16 103, label %case103
    i16 104, label %case104
    i16 105, label %case105
    i16 106, label %case106
    i16 107, label %case107
    i16 108, label %case108
    i16 109, label %case109
    i16 110, label %case110
    i16 111, label %case111
    i16 112, label %case112
    i16 113, label %case113
    i16 114, label %case114
    i16 115, label %case115
    i16 116, label %case116
    i16 117, label %case117
    i16 118, label %case118
    i16 119, label %case119
    i16 120, label %case120
    i16 121, label %case121
    i16 122, label %case122
    i16 123, label %case123
    i16 124, label %case124
    i16 125, label %case125
    i16 126, label %case126
    i16 127, label %case127
    i16 128, label %case128
    i16 129, label %case129
    i16 130, label %case130
    i16 131, label %case131
    i16 132, label %case132
    i16 133, label %case133
    i16 134, label %case134
    i16 135, label %case135
    i16 136, label %case136
    i16 137, label %case137
    i16 138, label %case138
    i16 139, label %case139
    i16 140, label %case140
    i16 141, label %case141
    i16 142, label %case142
    i16 143, label %case143
    i16 144, label %case144
    i16 145, label %case145
    i16 146, label %case146
    i16 147, label %case147
    i16 148, label %case148
    i16 149, label %case149
    i16 150, label %case150
    i16 151, label %case151
    i16 152, label %case152
    i16 153, label %case153
    i16 154, label %case154
    i16 155, label %case155
    i16 156, label %case156
    i16 157, label %case157
    i16 158, label %case158
    i16 159, label %case159
    i16 160, label %case160
    i16 161, label %case161
    i16 162, label %case162
    i16 163, label %case163
    i16 164, label %case164
    i16 165, label %case165
    i16 166, label %case166
    i16 167, label %case167
    i16 168, label %case168
    i16 169, label %case169
    i16 170, label %case170
    i16 171, label %case171
    i16 172, label %case172
    i16 173, label %case173
    i16 174, label %case174
    i16 175, label %case175
    i16 176, label %case176
    i16 177, label %case177
    i16 178, label %case178
    i16 179, label %case179
    i16 180, label %case180
    i16 181, label %case181
    i16 182, label %case182
    i16 183, label %case183
    i16 184, label %case184
    i16 185, label %case185
    i16 186, label %case186
    i16 187, label %case187
    i16 188, label %case188
    i16 189, label %case189
    i16 190, label %case190
    i16 191, label %case191
    i16 192, label %case192
    i16 193, label %case193
    i16 194, label %case194
    i16 195, label %case195
    i16 196, label %case196
    i16 197, label %case197
    i16 198, label %case198
    i16 199, label %case199
    i16 200, label %case200
    i16 201, label %case201
    i16 202, label %case202
    i16 203, label %case203
    i16 204, label %case204
    i16 205, label %case205
    i16 206, label %case206
    i16 207, label %case207
    i16 208, label %case208
    i16 209, label %case209
    i16 210, label %case210
    i16 211, label %case211
    i16 212, label %case212
    i16 213, label %case213
    i16 214, label %case214
    i16 215, label %case215
    i16 216, label %case216
    i16 217, label %case217
    i16 218, label %case218
    i16 219, label %case219
    i16 220, label %case220
    i16 221, label %case221
    i16 222, label %case222
    i16 223, label %case223
    i16 224, label %case224
    i16 225, label %case225
    i16 226, label %case226
    i16 227, label %case227
    i16 228, label %case228
    i16 229, label %case229
    i16 230, label %case230
    i16 231, label %case231
    i16 232, label %case232
    i16 233, label %case233
    i16 234, label %case234
    i16 235, label %case235
    i16 236, label %case236
    i16 237, label %case237
    i16 238, label %case238
    i16 239, label %case239
    i16 240, label %case240
    i16 241, label %case241
    i16 242, label %case242
    i16 243, label %case243
    i16 244, label %case244
    i16 245, label %case245
    i16 246, label %case246
    i16 247, label %case247
    i16 248, label %case248
    i16 249, label %case249
    i16 250, label %case250
    i16 251, label %case251
    i16 252, label %case252
    i16 253, label %case253
    i16 254, label %case254
    i16 255, label %case255
  ]

default:
  ret i16 65535
case0:
  ret i16 0
case1:
  ret i16 1
case2:
  ret i16 2
case3:
  ret i16 3
case4:
  ret i16 4
case5:
  ret i16 5
case6:
  ret i16 6
case7:
  ret i16 7
case8:
  ret i16 8
case9:
  ret i16 9
case10:
  ret i16 10
case11:
  ret i16 11
case12:
  ret i16 12
case13:
  ret i16 13
case14:
  ret i16 14
case15:
  ret i16 15
case16:
  ret i16 16
case17:
  ret i16 17
case18:
  ret i16 18
case19:
  ret i16 19
case20:
  ret i16 20
case21:
  ret i16 21
case22:
  ret i16 22
case23:
  ret i16 23
case24:
  ret i16 24
case25:
  ret i16 25
case26:
  ret i16 26
case27:
  ret i16 27
case28:
  ret i16 28
case29:
  ret i16 29
case30:
  ret i16 30
case31:
  ret i16 31
case32:
  ret i16 32
case33:
  ret i16 33
case34:
  ret i16 34
case35:
  ret i16 35
case36:
  ret i16 36
case37:
  ret i16 37
case38:
  ret i16 38
case39:
  ret i16 39
case40:
  ret i16 40
case41:
  ret i16 41
case42:
  ret i16 42
case43:
  ret i16 43
case44:
  ret i16 44
case45:
  ret i16 45
case46:
  ret i16 46
case47:
  ret i16 47
case48:
  ret i16 48
case49:
  ret i16 49
case50:
  ret i16 50
case51:
  ret i16 51
case52:
  ret i16 52
case53:
  ret i16 53
case54:
  ret i16 54
case55:
  ret i16 55
case56:
  ret i16 56
case57:
  ret i16 57
case58:
  ret i16 58
case59:
  ret i16 59
case60:
  ret i16 60
case61:
  ret i16 61
case62:
  ret i16 62
case63:
  ret i16 63
case64:
  ret i16 64
case65:
  ret i16 65
case66:
  ret i16 66
case67:
  ret i16 67
case68:
  ret i16 68
case69:
  ret i16 69
case70:
  ret i16 70
case71:
  ret i16 71
case72:
  ret i16 72
case73:
  ret i16 73
case74:
  ret i16 74
case75:
  ret i16 75
case76:
  ret i16 76
case77:
  ret i16 77
case78:
  ret i16 78
case79:
  ret i16 79
case80:
  ret i16 80
case81:
  ret i16 81
case82:
  ret i16 82
case83:
  ret i16 83
case84:
  ret i16 84
case85:
  ret i16 85
case86:
  ret i16 86
case87:
  ret i16 87
case88:
  ret i16 88
case89:
  ret i16 89
case90:
  ret i16 90
case91:
  ret i16 91
case92:
  ret i16 92
case93:
  ret i16 93
case94:
  ret i16 94
case95:
  ret i16 95
case96:
  ret i16 96
case97:
  ret i16 97
case98:
  ret i16 98
case99:
  ret i16 99
case100:
  ret i16 100
case101:
  ret i16 101
case102:
  ret i16 102
case103:
  ret i16 103
case104:
  ret i16 104
case105:
  ret i16 105
case106:
  ret i16 106
case107:
  ret i16 107
case108:
  ret i16 108
case109:
  ret i16 109
case110:
  ret i16 110
case111:
  ret i16 111
case112:
  ret i16 112
case113:
  ret i16 113
case114:
  ret i16 114
case115:
  ret i16 115
case116:
  ret i16 116
case117:
  ret i16 117
case118:
  ret i16 118
case119:
  ret i16 119
case120:
  ret i16 120
case121:
  ret i16 121
case122:
  ret i16 122
case123:
  ret i16 123
case124:
  ret i16 124
case125:
  ret i16 125
case126:
  ret i16 126
case127:
  ret i16 127
case128:
  ret i16 128
case129:
  ret i16 129
case130:
  ret i16 130
case131:
  ret i16 131
case132:
  ret i16 132
case133:
  ret i16 133
case134:
  ret i16 134
case135:
  ret i16 135
case136:
  ret i16 136
case137:
  ret i16 137
case138:
  ret i16 138
case139:
  ret i16 139
case140:
  ret i16 140
case141:
  ret i16 141
case142:
  ret i16 142
case143:
  ret i16 143
case144:
  ret i16 144
case145:
  ret i16 145
case146:
  ret i16 146
case147:
  ret i16 147
case148:
  ret i16 148
case149:
  ret i16 149
case150:
  ret i16 150
case151:
  ret i16 151
case152:
  ret i16 152
case153:
  ret i16 153
case154:
  ret i16 154
case155:
  ret i16 155
case156:
  ret i16 156
case157:
  ret i16 157
case158:
  ret i16 158
case159:
  ret i16 159
case160:
  ret i16 160
case161:
  ret i16 161
case162:
  ret i16 162
case163:
  ret i16 163
case164:
  ret i16 164
case165:
  ret i16 165
case166:
  ret i16 166
case167:
  ret i16 167
case168:
  ret i16 168
case169:
  ret i16 169
case170:
  ret i16 170
case171:
  ret i16 171
case172:
  ret i16 172
case173:
  ret i16 173
case174:
  ret i16 174
case175:
  ret i16 175
case176:
  ret i16 176
case177:
  ret i16 177
case178:
  ret i16 178
case179:
  ret i16 179
case180:
  ret i16 180
case181:
  ret i16 181
case182:
  ret i16 182
case183:
  ret i16 183
case184:
  ret i16 184
case185:
  ret i16 185
case186:
  ret i16 186
case187:
  ret i16 187
case188:
  ret i16 188
case189:
  ret i16 189
case190:
  ret i16 190
case191:
  ret i16 191
case192:
  ret i16 192
case193:
  ret i16 193
case194:
  ret i16 194
case195:
  ret i16 195
case196:
  ret i16 196
case197:
  ret i16 197
case198:
  ret i16 198
case199:
  ret i16 199
case200:
  ret i16 200
case201:
  ret i16 201
case202:
  ret i16 202
case203:
  ret i16 203
case204:
  ret i16 204
case205:
  ret i16 205
case206:
  ret i16 206
case207:
  ret i16 207
case208:
  ret i16 208
case209:
  ret i16 209
case210:
  ret i16 210
case211:
  ret i16 211
case212:
  ret i16 212
case213:
  ret i16 213
case214:
  ret i16 214
case215:
  ret i16 215
case216:
  ret i16 216
case217:
  ret i16 217
case218:
  ret i16 218
case219:
  ret i16 219
case220:
  ret i16 220
case221:
  ret i16 221
case222:
  ret i16 222
case223:
  ret i16 223
case224:
  ret i16 224
case225:
  ret i16 225
case226:
  ret i16 226
case227:
  ret i16 227
case228:
  ret i16 228
case229:
  ret i16 229
case230:
  ret i16 230
case231:
  ret i16 231
case232:
  ret i16 232
case233:
  ret i16 233
case234:
  ret i16 234
case235:
  ret i16 235
case236:
  ret i16 236
case237:
  ret i16 237
case238:
  ret i16 238
case239:
  ret i16 239
case240:
  ret i16 240
case241:
  ret i16 241
case242:
  ret i16 242
case243:
  ret i16 243
case244:
  ret i16 244
case245:
  ret i16 245
case246:
  ret i16 246
case247:
  ret i16 247
case248:
  ret i16 248
case249:
  ret i16 249
case250:
  ret i16 250
case251:
  ret i16 251
case252:
  ret i16 252
case253:
  ret i16 253
case254:
  ret i16 254
case255:
  ret i16 255
}
