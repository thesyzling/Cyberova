import tkinter as tk
from tkinter import messagebox

def check_winner():
    
    for i in range(3):
        # Satır kontrolü
        if buttons[i][0]["text"] == buttons[i][1]["text"] == buttons[i][2]["text"] and buttons[i][0]["text"] != "":
            return buttons[i][0]["text"]
        # Sütun kontrolü
        if buttons[0][i]["text"] == buttons[1][i]["text"] == buttons[2][i]["text"] and buttons[0][i]["text"] != "":
            return buttons[0][i]["text"]
    
    # Çapraz kontrol
    if buttons[0][0]["text"] == buttons[1][1]["text"] == buttons[2][2]["text"] and buttons[0][0]["text"] != "":
        return buttons[0][0]["text"]
    if buttons[0][2]["text"] == buttons[1][1]["text"] == buttons[2][0]["text"] and buttons[0][2]["text"] != "":
        return buttons[0][2]["text"]
    
    return None

def check_draw():
    
    for row in buttons:
        for button in row:
            if button["text"] == "":
                return False
    return True

def on_button_click(row, col):
    
    global current_player
    
    if buttons[row][col]["text"] == "":
        buttons[row][col]["text"] = current_player
        winner = check_winner()
        
        if winner:
            messagebox.showinfo("Oyun Bitti", f"{winner} kazandı!")
            reset_board()
        elif check_draw():
            messagebox.showinfo("Oyun Bitti", "Oyun berabere!")
            reset_board()
        else:
            # Oyuncu değiştir
            current_player = "O" if current_player == "X" else "X"
    else:
        messagebox.showwarning("Geçersiz Hamle", "Bu hücre zaten dolu!")

def reset_board():
    """Oyunu sıfırlar."""
    global current_player
    current_player = "X"
    for row in buttons:
        for button in row:
            button["text"] = ""

# Ana Pencere
root = tk.Tk()
root.title("XOX Oyunu Melisa")

# Oyuncu sırası
current_player = "X"

# Butonlar için 3x3 liste
buttons = [[None for _ in range(3)] for _ in range(3)]

# Butonları oluştur
for i in range(3):
    for j in range(3):
        button = tk.Button(root, text="", font=("Arial", 24), width=5, height=2,
                           command=lambda row=i, col=j: on_button_click(row, col))
        button.grid(row=i, column=j)
        buttons[i][j] = button

# Oyunu başlat
root.mainloop()

