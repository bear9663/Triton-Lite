import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import time

class ArduinoController:
    def __init__(self, root):
        self.root = root
        self.root.title("Triton-Lite コントローラー")
        
        # シリアル接続
        self.serial = None
        
        # メインウィンドウサイズ設定
        self.root.geometry("600x800")
        
        # GUI作成
        self.create_widgets()
        
    def create_widgets(self):
        # メインフレームの作成
        main_frame = ttk.Frame(self.root)
        main_frame.grid(row=0, column=0, sticky="nsew")
        
        # 接続設定フレーム
        connect_frame = ttk.LabelFrame(main_frame, text="接続設定", padding=10)
        connect_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")
        
        # COMポート選択
        ttk.Label(connect_frame, text="ポート:").grid(row=0, column=0, padx=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(connect_frame, textvariable=self.port_var)
        self.port_combo.grid(row=0, column=1, padx=5)
        
        # 接続ボタン
        self.connect_btn = ttk.Button(connect_frame, text="接続", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=2, padx=5)
        
        # ポート更新ボタン
        refresh_btn = ttk.Button(connect_frame, text="ポート更新", command=self.update_ports)
        refresh_btn.grid(row=0, column=3, padx=5)
        
        # 設定フレーム
        settings_frame = ttk.LabelFrame(main_frame, text="動作時間設定", padding=10)
        settings_frame.grid(row=1, column=0, padx=10, pady=5, sticky="ew")
        
        # 浮上時間設定
        ttk.Label(settings_frame, text="浮上時間 (秒):").grid(row=0, column=0, padx=5, pady=5)
        self.sup_start_var = tk.StringVar(value="30")
        self.sup_start_entry = ttk.Entry(settings_frame, textvariable=self.sup_start_var)
        self.sup_start_entry.grid(row=0, column=1, padx=5, pady=5)
        
        # 潜水時間設定
        ttk.Label(settings_frame, text="潜水時間 (秒):").grid(row=1, column=0, padx=5, pady=5)
        self.exh_start_var = tk.StringVar(value="30")
        self.exh_start_entry = ttk.Entry(settings_frame, textvariable=self.exh_start_var)
        self.exh_start_entry.grid(row=1, column=1, padx=5, pady=5)
        
        # 説明フレーム
        desc_frame = ttk.LabelFrame(main_frame, text="説明", padding=10)
        desc_frame.grid(row=2, column=0, padx=10, pady=5, sticky="ew")
        ttk.Label(desc_frame, 
                 text="浮上時間：浮上開始までの待機時間\n"
                      "潜水時間：潜水開始までの待機時間\n"
                      "※設定後は必ず「設定を適用」を押してください").grid(
            row=0, column=0, padx=5, pady=5)
        
        # 適用ボタン
        apply_btn = ttk.Button(settings_frame, text="設定を適用", command=self.apply_settings)
        apply_btn.grid(row=2, column=0, columnspan=2, pady=10)
        
        # ステータス表示
        self.status_var = tk.StringVar(value="未接続")
        status_label = ttk.Label(main_frame, textvariable=self.status_var)
        status_label.grid(row=3, column=0, pady=5)
        
        # 通信ログフレーム
        log_frame = ttk.LabelFrame(main_frame, text="通信ログ", padding=10)
        log_frame.grid(row=4, column=0, padx=10, pady=5, sticky="ew")
        
        # 通信ログテキストボックス
        self.log_text = tk.Text(log_frame, height=15, width=60)
        self.log_text.grid(row=0, column=0, padx=5, pady=5)
        
        # スクロールバー
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scrollbar.set)
        
        # ログクリアボタン
        clear_log_btn = ttk.Button(log_frame, text="ログクリア", command=self.clear_log)
        clear_log_btn.grid(row=1, column=0, pady=5)
        
        # 初期化時にポート更新
        self.update_ports()
        
    def clear_log(self):
        """ログをクリア"""
        self.log_text.delete(1.0, tk.END)
        
    def log_message(self, message):
        """ログにメッセージを追加"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        
    def update_ports(self):
        """利用可能なCOMポートを更新"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.set(ports[0])
        self.log_message(f"利用可能なポート: {', '.join(ports)}")
        
    def toggle_connection(self):
        """接続/切断の切り替え"""
        if self.serial is None:
            try:
                port = self.port_var.get()
                self.serial = serial.Serial(port, 9600, timeout=1)
                self.status_var.set("接続済み")
                self.connect_btn.config(text="切断")
                self.log_message(f"ポート {port} に接続しました")
                
            except serial.SerialException as e:
                messagebox.showerror("エラー", f"接続エラー: {str(e)}")
                self.log_message(f"接続エラー: {str(e)}")
                return
        else:
            self.serial.close()
            self.serial = None
            self.status_var.set("未接続")
            self.connect_btn.config(text="接続")
            self.log_message("切断しました")
        
    def apply_settings(self):
        """設定をArduinoに送信"""
        if self.serial is None:
            messagebox.showerror("エラー", "Arduinoに接続されていません")
            self.log_message("エラー: Arduinoに接続されていません")
            return
            
        try:
            # 浮上時間設定送信
            sup_command = f"SUP_START:{self.sup_start_var.get()}\n"
            self.serial.write(sup_command.encode())
            self.log_message(f"送信: {sup_command.strip()}")
            
            # 応答を待つ
            time.sleep(0.1)
            while self.serial.in_waiting:
                response = self.serial.readline().decode().strip()
                self.log_message(f"Arduino応答: {response}")
            
            # 潜水時間設定送信
            exh_command = f"EXH_START:{self.exh_start_var.get()}\n"
            self.serial.write(exh_command.encode())
            self.log_message(f"送信: {exh_command.strip()}")
            
            # 応答を待つ
            time.sleep(0.1)
            while self.serial.in_waiting:
                response = self.serial.readline().decode().strip()
                self.log_message(f"Arduino応答: {response}")
            
            messagebox.showinfo("成功", "設定を更新しました")
            self.log_message("設定更新完了")
            
        except serial.SerialException as e:
            messagebox.showerror("エラー", f"通信エラー: {str(e)}")
            self.log_message(f"通信エラー: {str(e)}")

if __name__ == "__main__":
    root = tk.Tk()
    app = ArduinoController(root)
    root.mainloop()