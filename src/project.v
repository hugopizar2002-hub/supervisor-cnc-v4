`default_nettype none

module tt_um_supervisor_cnc_v4 (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    supervisor_cnc_v4 supervisor (
        .clk           (clk),
        .rst           (~rst_n),
        .step_x        (ui_in[0]),
        .step_y        (ui_in[1]),
        .vigilar_x     (ui_in[2]),
        .vigilar_y     (ui_in[3]),
        .falla_x       (uo_out[0]),
        .falla_y       (uo_out[1]),
        .falla_general (uo_out[2]),
        .salida_segura (uo_out[3])
    );

    assign uo_out[7:4] = 4'b0000;
    assign uio_out     = 8'b00000000;
    assign uio_oe      = 8'b00000000;

    wire _unused = &{ena, ui_in[7:4], uio_in, 1'b0};

endmodule

// Supervisor CNC V4 - Silicluster V3 (Verilog-2001)
// Entradas y salidas logicas activas en alto, salvo indicacion de rst.
// CLK = 10 MHz. Divisor compartido: 10,000 ciclos = 1 ms.
// Dos contadores de 2,000 ticks = 2 segundos sin pulsos.
module supervisor_cnc_v4 #(
    parameter integer PRESCALE = 10000,
    parameter integer TIMEOUT_TICKS = 2000
)(
    input  wire clk,
    input  wire rst,
    input  wire step_x,
    input  wire step_y,
    input  wire vigilar_x,
    input  wire vigilar_y,
    output wire falla_x,
    output wire falla_y,
    output wire falla_general,
    output wire salida_segura
);
    // Dimensionados para los valores por defecto de 10 MHz / 2 s.
    // Para la simulacion se modifican solo los parametros, no los anchos.
    reg [13:0] divisor;
    wire tick_ms = (divisor == PRESCALE - 1);

    reg [10:0] tiempo_x;
    reg [10:0] tiempo_y;
    reg fx, fy;

    // Sincronizacion de entradas asincronas al dominio clk.
    reg x_meta, x_sync, x_prev;
    reg y_meta, y_sync, y_prev;
    reg vx_meta, vx_sync;
    reg vy_meta, vy_sync;

    wire pulso_x = x_sync & ~x_prev;
    wire pulso_y = y_sync & ~y_prev;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            divisor <= 14'd0;
            tiempo_x <= 11'd0;
            tiempo_y <= 11'd0;
            fx <= 1'b0;
            fy <= 1'b0;
            x_meta <= 1'b0; x_sync <= 1'b0; x_prev <= 1'b0;
            y_meta <= 1'b0; y_sync <= 1'b0; y_prev <= 1'b0;
            vx_meta <= 1'b0; vx_sync <= 1'b0;
            vy_meta <= 1'b0; vy_sync <= 1'b0;
        end else begin
            x_meta <= step_x;  x_sync <= x_meta;  x_prev <= x_sync;
            y_meta <= step_y;  y_sync <= y_meta;  y_prev <= y_sync;
            vx_meta <= vigilar_x; vx_sync <= vx_meta;
            vy_meta <= vigilar_y; vy_sync <= vy_meta;

            if (tick_ms)
                divisor <= 14'd0;
            else
                divisor <= divisor + 1'b1;

            // Las fallas son retenidas hasta reset.
            if (!fx) begin
                if (!vx_sync || pulso_x)
                    tiempo_x <= 11'd0;
                else if (tick_ms) begin
                    if (tiempo_x == TIMEOUT_TICKS - 1)
                        fx <= 1'b1;
                    else
                        tiempo_x <= tiempo_x + 1'b1;
                end
            end

            if (!fy) begin
                if (!vy_sync || pulso_y)
                    tiempo_y <= 11'd0;
                else if (tick_ms) begin
                    if (tiempo_y == TIMEOUT_TICKS - 1)
                        fy <= 1'b1;
                    else
                        tiempo_y <= tiempo_y + 1'b1;
                end
            end
        end
    end

    assign falla_x = fx;
    assign falla_y = fy;
    assign falla_general = fx | fy;
    // 1 = sin falla detectada; 0 = alarma. SOLO INDICACION LOGICA.
    assign salida_segura = ~(fx | fy);
endmodule

`default_nettype wire
