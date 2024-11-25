import random

def boardBas(board):
    
    for row in board:
        print(" | ".join(row))
        print("-" * 9)


def kazandi_mi(board, sembol):
    
    
    for i in range(3):
        if all(board[i][j] == sembol for j in range(3)):  
            return True
        if all(board[j][i] == sembol for j in range(3)):  
            return True
    
    if all(board[i][i] == sembol for i in range(3)) or all(board[i][2 - i] == sembol for i in range(3)):
        return True
    return False


def boardKontrol(board):
    
    return all(cell != " " for row in board for cell in row)


def bilgisayarHamlesi(board, bilgisayar, oyuncu):
    
    
    for i in range(3):
        for j in range(3):
            if board[i][j] == " ":
                
                board[i][j] = bilgisayar
                if kazandi_mi(board, bilgisayar):
                    return board  
                board[i][j] = " "  

    
    for i in range(3):
        for j in range(3):
            if board[i][j] == " ":
                
                board[i][j] = oyuncu
                if kazandi_mi(board, oyuncu):
                    board[i][j] = bilgisayar  
                    return board
                board[i][j] = " "  

    
    if board[1][1] == " ":
        board[1][1] = bilgisayar
        return board

    
    boşKareler = [(i, j) for i in range(3) for j in range(3) if board[i][j] == " "]
    if boşKareler:
        i, j = random.choice(boşKareler)
        board[i][j] = bilgisayar
        return board

    return board  


def oyuncuHamlesi(board):
   
    while True:
        try:
            print("Hamlenizin satir ve sutununu girin (0-2): ")
            satir, sutun = map(int, input().split())
            if board[satir][sutun] == " ":
                return satir, sutun
            else:
                print("Bu kare dolu! Baska bir kare secin.")
        except (ValueError, IndexError):
            print("Gecersiz giris! Satir ve sutun degerleri 0, 1 veya 2 olmalidir.")


def oyun():
 
    
    board = [[" " for _ in range(3)] for _ in range(3)]

    print("XOX Oyununa Hos Geldiniz!")
    print("Oynamak istediginiz sembolü secin (X veya O):")
    oyuncu = input().upper()

    while oyuncu not in ["X", "O"]:
        print("Gecersiz secim! Lutfen 'X' veya 'O' secin:")
        oyuncu = input().upper()

    bilgisayar = "O" if oyuncu == "X" else "X"
    print(f"Siz: {oyuncu}, Bilgisayar: {bilgisayar}")
    
    boardBas(board)

    
    sira = "oyuncu" if oyuncu == "X" else "bilgisayar"

    while True:
        if sira == "oyuncu":
            print("\nOyuncunun sirasi:")
            satir, sutun = oyuncuHamlesi(board)
            board[satir][sutun] = oyuncu
            boardBas(board)
            if kazandi_mi(board, oyuncu):
                print("Tebrikler kazandiniz!")
                break
            sira = "bilgisayar"
        else:
            print("\nBilgisayarin sirasi:")
            board = bilgisayarHamlesi(board, bilgisayar, oyuncu)
            boardBas(board)
            if kazandi_mi(board, bilgisayar):
                print("Bilgisayar kazandi!")
                break
            sira = "oyuncu"

        
        if boardKontrol(board):
            print("Berabere!")
            break

oyun()