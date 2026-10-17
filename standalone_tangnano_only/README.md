# AdaReMAC — bản chỉ Tang Nano 20K (chưa cần ESP32-S3)

**Thư mục này tự chứa đủ mọi file cần — add TẤT CẢ file .v trong đây vào Gowin EDA,
không cần lấy thêm gì từ nơi khác.**

## 1. Kiểm tra trước khi nạp (khuyến khích, cần iverilog: `sudo apt install iverilog`)
```
iverilog -g2005 -o /tmp/tb tb_standalone.v top_standalone.v report_tx.v uart_tx.v btn_sync.v mac_array.v adaremac_mac_lane.v adaremac_mul.v
vvp /tmp/tb
```
Phải thấy dòng cuối `TB_STANDALONE PASS`. Nếu không thấy — đừng nạp lên board, báo lại lỗi in ra.

## 2. Gowin EDA
1. New Project → device **GW2AR-LV18QN88C8/I7**.
2. Add **toàn bộ 7 file .v** trong thư mục này (top module = `top_standalone`).
3. Add `tn20k_standalone.cst` và `tn20k_standalone.sdc`.
4. Synthesize → Place & Route → Programmer → SRAM Program.

## 3. Đọc UART — làm đúng thứ tự này, Tang Nano 20K hơi đặc biệt
Board dùng chip BL616 làm cầu USB↔UART, và theo tài liệu chính hãng **mặc định
nó đã ở "uart mode" (chế độ truyền thẳng)** — nhưng nếu lỡ tay gõ Ctrl+C trong
terminal, board có thể rơi vào **menu lệnh riêng của BL616** (dùng để test LED,
chỉnh clock...) — lúc đó gõ gì cũng không thấy dữ liệu từ FPGA, trông giống
"serial không đọc được gì" nhưng thực ra là đang kẹt trong menu khác.

**Checklist khi không thấy gì:**
1. Thiết bị quản lý cổng (Device Manager / `ls /dev/tty*`) thường hiện **2 cổng COM**
   khi cắm board — thử cả hai, không chỉ cổng số nhỏ hơn.
2. Đóng Gowin Programmer / FPGA Companion lại trước khi mở terminal — phần mềm
   nạp có thể đang giữ cổng, làm terminal khác không mở được hoặc im lặng.
3. Baud **115200, 8N1, không flow control**. Sai baud thường ra toàn ký tự rác,
   không phải im lặng hoàn toàn — im lặng tuyệt đối thường là sai cổng hoặc chưa nạp xong.
4. Nếu mở đúng cổng mà vẫn im lặng, gõ thử `Enter` vài lần — nếu board phản hồi
   một dòng menu dạng `>` hoặc danh sách lệnh, đó là **menu BL616**, không phải
   FPGA. Thử cắm rút lại cáp USB để mở cổng mới sạch, tránh việc terminal cũ
   đang ở sai mode.
5. Nhấn thử **S2** (không phải S1) — nếu đổi cấu hình mà LED0-2 đổi theo nhưng
   UART vẫn im, khả năng cao là sai cổng COM chứ không phải lỗi thiết kế.

## 4. Vận hành
- Mỗi ~1 giây in 1 dòng: `S<n> OPS=... ENR=... ERR=...` (tất cả số hex).
- Bấm **S2** để đổi cấu hình 0→1→2→3→4→0, in báo cáo ngay khi đổi.
- Bấm giữ **S1** = reset (dừng hẳn, OPS đứng yên), thả ra chạy tiếp từ đầu.
- NMED = ERR / (OPS × 16384).

| cfg | mode | thr | Ý nghĩa |
|---|---|---|---|
| 0 | 0 | 0 | Full Approximate |
| 1 | 1 | 3 | Selective, ngưỡng cao → residual bật ít |
| 2 | 1 | 2 | Selective, ngưỡng trung |
| 3 | 1 | 1 | Selective, ngưỡng thấp → residual bật nhiều |
| 4 | 2 | 0 | Full Exact (tham chiếu đúng tuyệt đối) |
