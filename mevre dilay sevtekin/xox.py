#   xox oyunu
oyuntahtasi=[ ' ' for i in range(10)]
def yazdir():
    print(' ',oyuntahtasi[1] ,' ','|',' ',oyuntahtasi[2] ,' ','|',' ',oyuntahtasi[3] )
    print("------------------")
    print(' ',oyuntahtasi[4] ,' ','|',' ',oyuntahtasi[5] ,' ','|',' ',oyuntahtasi[6] )
    print("------------------")
    print(' ',oyuntahtasi[7] ,' ','|',' ',oyuntahtasi[8] ,' ','|',' ',oyuntahtasi[9] )

def harfkoy(harf,konum):
    oyuntahtasi[konum]=harf

def alanbos_mu(konum):
    return oyuntahtasi[konum]==' '
def tahtadolu_mu():
    if oyuntahtasi.count(' ')>1:
        return False
    else:
        return True
def kazanan(oyuntahtasi,harf):
   return(oyuntahtasi[1]==harf and oyuntahtasi[2]==harf and oyuntahtasi[3]==harf 
   or oyuntahtasi[4]==harf and oyuntahtasi[5]==harf and oyuntahtasi[6]==harf 
   or oyuntahtasi[7]==harf and oyuntahtasi[8]==harf and oyuntahtasi[9]==harf
   or oyuntahtasi[1]==harf and oyuntahtasi[4]==harf and oyuntahtasi[7]==harf 
   or oyuntahtasi[2]==harf and oyuntahtasi[5]==harf and oyuntahtasi[8]==harf
   or oyuntahtasi[3]==harf and oyuntahtasi[6]==harf and oyuntahtasi[9]==harf 
   or oyuntahtasi[1]==harf and oyuntahtasi[5]==harf and oyuntahtasi[9]==harf
   or oyuntahtasi[3]==harf and oyuntahtasi[5]==harf and oyuntahtasi[7]==harf  )   

def oyuncu_hareket():
    konum=int(input("1-9 arasinda bir deger giriniz :"))
    if alanbos_mu(konum):
        harfkoy("x",konum)
        if kazanan(oyuntahtasi,"x"):
            yazdir()
            print("tebrikler kazandiniz!!")
            exit()
    
        yazdir()
    else:   
        print("girdiginiz alan dolu tekrar secin") 
        oyuncu_hareket()
def pc_hareket():
    import random
    musait_konumlar=[konum for konum,harf in enumerate(oyuntahtasi) if harf==" " and konum !=0  ]

    hareket=0
    for harf in ['o','x']:
        for i in musait_konumlar:
            kopya_tahta=oyuntahtasi[:]
            kopya_tahta[i]=harf
            if kazanan(kopya_tahta,harf):
                hareket=i
                return hareket
    koseler=[]
    for i in musait_konumlar:
        if i in [1,3,7,9]:  
            koseler.append(i)
    if len(koseler)> 0:   
        hareket =random.choice(koseler)
        return hareket

    if 5 in musait_konumlar:
        hareket=5
        return hareket
    icerisi=[]
    for i in musait_konumlar:
        if i in [2,4,6,8]:  
            icerisi.append(i)
    if len(icerisi)> 0:   
        hareket =random.choice(icerisi)
        return hareket

def oyun():
    print("xox oyununa hosgeldiniz")
    yazdir()

    while not tahtadolu_mu():
        oyuncu_hareket()
        if tahtadolu_mu():
            print("oyun bitti kazanan yok")
            exit()
        print("-----------------------")  

        pc_hamle= pc_hareket() 
        harfkoy('o',pc_hamle)
        if kazanan(oyuntahtasi,'o'):
            yazdir()
            print("bilgisayar kazandi")
            exit()
        yazdir()
        if tahtadolu_mu():
            print("oyun bitti kazanan yok")
            exit()
        print("-----------------------")  
oyun()