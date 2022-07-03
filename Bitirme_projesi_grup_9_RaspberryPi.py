import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
import bluetooth

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)   # hata mesajı kapatıldı

target_name = "HC-06" #Arduino uzerinde kullanılan bluetooth modulu
target_address = None #modulun mac adresi
nearby_devices = bluetooth.discover_devices() #bluetooth etraftaki cihazları aramaya basladi

for bdaddr in nearby_devices:# cihaz bulmak icin loop a girildi
    if target_name == bluetooth.lookup_name( bdaddr ):
        target_address = bdaddr
        break
    
port = 1 #bluetooth icin arduino uzeinden port secildi
sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM ) #bluetooth komutlari icin obje tanımlandı
sock.connect((target_address, port))  #obje mac adresimize atandi  

servoPIN = 4
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)#ilk servo icin raspberry pi uzerinde 4. pin secildi

servoPIN_2 = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN_2, GPIO.OUT)#ikinci servo icin raspberry pi uzerinde 27. pin secildi

p = GPIO.PWM(servoPIN, 50) # GPIO 4 for PWM with 50Hz
p.start(7.5) # Initialization

p_2 = GPIO.PWM(servoPIN_2, 50) # GPIO 27 for PWM with 50Hz
p_2.start(7.5) # Initialization

#goruntu isleme icin koordinat degiskenleri
x=0   # cisim x koordinat için değişken
y=0   # cisim y koordinat için değişken

cap = cv2.VideoCapture(0)  # alınan video cap değişkeine atandı


cap.set(3, 600)
cap.set(3, 600) # cap değişkenine atanan videonun boyutu belirlendi
_, frame = cap.read()
rows, cols, _ = frame.shape

x_medium = int(cols / 2)
y_medium = int(cols / 2)
w_medium = int(cols / 2)
h_medium = int(cols / 2)
center = int(cols / 2)     # cizme ait x y h w bilgileri değişkenlere atandı

k_p_x = 19 #x ekseni icin Kp degeri
k_i_x = 0.13126 #x ekseni icin Ki degeri
k_d_x = 223 #x ekseni icin Kd degeri
k_p_y = 19 #y ekseni icin Kp degeri
k_i_y = 0.13126 #y ekseni icin Ki degeri
k_d_y = 223 #y ekseni icin Ki degeri
#hata degerleri kontrolcu blogundan gecince toplanan degiskenler belirlendi
PID_sum_x = 0 
PID_sum_y = 0

zero_point_x = 305#x koordinati icin tahtanın ortasının konumu
zero_point_y = 211#y koordinati icin tahtanın ortasının konumu
prev_error_x = 0#PID kontrolcunun turev isleminde kullanılacak onceki hata degeri
prev_error_y = 0#her while dongusu sonunda hata degeri bu degiskene atanır
PID_p_x = 0#x koordinatı icin orantısal kontrol degeri
PID_p_y = 0#y koordinati icin orantısal kontrol degeri
PID_i_x = 0#x koordinatı icin integral kontrol degeri
PID_i_y = 0#y koordinatı icin integral kontrol degeri
PID_d_x = 0#x koordinatı icin türev kontrol degeri
PID_d_y = 0#y koordinatı icin turev kontrol degeri
pointOfInt = 500#integral kontrolun devreye girme capi
adjPID_x = 0#x PID toplam degerinin servo motorun duty cycle degerine orantılama degiskeni
adjPID_y = 0#y PID toplam degerinin servo motorun duty cycle degerine orantılama degiskeni
DS_x = 0#duty cycle x
DS_y = 0#duty cycle y

while True: # PID kontrol ve goruntu isleme icin sonsuz döngüye girildi 
    
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # renk algilama işlemi başlatıldı
    
    #low_red = np.array([161,155,84])    # alt kırımızı rengin bilgileri girildi
    #high_red = np.array([179, 255, 255])  # üst kırımızı rengin bilgileri girildi
    #low_red = np.array([60, 70, 50])    # alt yesil rengin bilgileri girildi
    #high_red = np.array([110, 255, 255])  # üst yesil rengin bilgileri girildi
    #low_red = np.array([100,150,0])    # alt mavi rengin bilgileri girildi
    #high_red = np.array([140, 255, 255])  # üst mavi rengin bilgileri girildi
    #low_red = np.array([79,79,47])    # alt gri rengin bilgileri girildi
    #high_red = np.array([220, 220, 220])  # üst gri rengin bilgileri girildi
    low_red = np.array([5,50,50])    # alt turuncu rengin bilgileri girildi
    high_red = np.array([15,255,255])  # üst turuncu rengin bilgileri girildi
    #low_red  = np.array([20, 100, 100])   # alt sarı rengin bilgileri girildi
    #high_red = np.array([30,255, 255]) # üst sarı rengin bilgileri girildi

    
    red_mask = cv2.inRange(hsv_frame, low_red, high_red)  # renk bilgileri değişkenen atandı
    _, contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)    # renk algılanırsa işleme devam edilecek
    
    for cnt in contours:
        (x, y, w, h) = cv2.boundingRect(cnt) # algılanan rengin x ve y koordinatları bulundu        
        x_medium = int((x + x + w) / 2)
        break
                                            # boy      renk  kalinlik
    cv2.line(frame, (x_medium, 0), (x_medium, 480), (0, 0, 255), 2)  # x koordinatı ekranda gösterildi
    
    cv2.imshow("Frame", frame)
    error_x = zero_point_x - x #x koordinatı orta noktadan sapma degeri, hata
    error_y = y-zero_point_y #y koordinatı orta noktadan sapma degeri, hata
    
    #seri haberlesme icin time kullanildi, x ve y koordinat degerleri gonderildi   
    start = time.time()
    a = round(start)*1000
    if a % 300 == 0:
     sock.send(bytes(str(error_x), 'utf8'))
     sock.send("y")
     sock.send(":")
     sock.send(bytes(str(error_y), 'utf8'))
     sock.send("x")
     sock.send(":")

    PID_p_x = k_p_x*error_x #x icin orantısal kontrol
    PID_p_y = k_p_y*error_y #y icin oranstısal kontrol
    PID_d_x = k_d_x*(error_x - prev_error_x) #x icin turev kontrol
    PID_d_y = k_d_y*(error_y - prev_error_y) #y icin turev kontrol
    if error_x>pointOfInt:
     PID_i_x = 0#eger top merkeze belli bir yakınlıkta degilse integral kontrol yapılmaz
    else :
     PID_i_x = PID_i_x + (k_i_x * error_x)#x icin integral kontrol
    if error_y>pointOfInt:
     PID_i_y = 0#eger top merkeze belli bir yakınlıkta degilse integral kontrol yapılmaz
    else :
     PID_i_y = PID_i_y + (k_i_y * error_y)#y icin integral kontrol
    PID_sum_x = PID_p_x + PID_d_x + PID_i_x#kontrol degerleri toplandı x
    PID_sum_y = PID_p_y + PID_d_y + PID_i_y#kontrol degerleri toplandı y
    #kontrol degerleri -10000 ve 10000 arasında degerlere sinirlandi
    if PID_sum_x > 10000:
        PID_sum_x = 10000
    elif PID_sum_x < -10000:
        PID_sum_x = -10000
    if PID_sum_y > 10000:
        PID_sum_y = 10000
    elif PID_sum_y < -10000:
        PID_sum_y = -10000    
    prev_error_x = error_x#turev kontrol icin simdi ki hata önceki hataya esitlendi
    prev_error_y = error_y
    #print("pid x  ",PID_sum_x,"pid y  ",PID_sum_y)
    print("x ",error_x,"y ",error_y)#konum konsola basildi
    #print("duty cycle x: ",DS_x)
    #print("duty cycle y: ",DS_y)
    #print("x ",x," y ",y)
    adjPID_x = PID_sum_x + 10000
    DS_x = 2.5 + adjPID_x*0.0005
    #PID degerleri, servo motor duty cycle icin orantılandı
    p.ChangeDutyCycle(DS_x)#servo motorun icine kontrol blogundan gelen degerler atandı
    adjPID_y = PID_sum_y + 10000
    DS_y = 2.5 + adjPID_y*0.0005
    #PID degerleri, servo motor duty cycle icin orantılandı
    p_2.ChangeDutyCycle(DS_y)#servo motorun icine kontrol blogundan gelen degerler atandı
    key = cv2.waitKey(1) 
    
    if key == 27:   # renk olup olmadığı sorgulandı
        break
       
    

#cap.release()
#cv2.destroyAllWindows()



