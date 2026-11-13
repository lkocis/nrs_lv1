Pose 1:
seed: 5
Ormarić pose = [0.012, -0.726, 0.178, 1.866], axis_pos = -1, theta0 = 20.0 deg
Gotovo: vrata otvorena na ~80.0 deg
USPJEH!

Pose 2:
seed: 11
Ormarić pose = [-0.005, -0.744, 0.178, 1.995], axis_pos = -1, theta0 = 20.0 deg
Gotovo: vrata otvorena na ~80.0 deg
USPJEH!

Pose 3:
seed: 20
Ormarić pose = [0.041, -0.731, 0.178, 1.837], axis_pos = -1, theta0 = 20.0 deg
Gotovo: vrata otvorena na ~80.0 deg
USPJEH!

Pose 4:
seed: 30
Ormarić pose = [0.004, -0.771, 0.178, 1.101], axis_pos = -1, theta0 = 20.0 deg
Gotovo: vrata otvorena na ~80.0 deg
USPJEH!

Pose 5:
seed: 42
Ormarić pose = [0.014, -0.797, 0.178, 1.346], axis_pos = -1, theta0 = 20.0 deg
Gotovo: vrata otvorena na ~80.0 deg
USPJEH!

Natuknice za odgovaranje:
1) Kako odrediti os prsta koji hvata vrata i koordinate TCP-a?
Korak 1: pogledaj šaku u RVizu i nađi prst A, u RVizu dodaj prikaz TF i u njemu uključi samo okvire robotiq_3f_frame i AGS_finger-dist002. Boje osi: crvena = x, zelena = y, plava = z.

Korak 2: Položaj (xyz) određuje se tako da pokrenem RViz, kliknem na jastučić 
Gore lijevo, u Global Options → Fixed Frame, upiši robotiq_3f_frame. Sve koordinate tada su u okviru šake.
U drugom terminalu pokreni:
bash
   ros2 topic echo /clicked_point
U RVizu na alatnoj traci odaberi alat Publish Point i klikni na jastučić prsta A (unutarnja ploha), malo prije vrha prsta.
U terminalu se ispiše točka, npr.:
   point:
     x: 0.078
     y: 0.19
     z: 0.0
To je xyz. 

2) 
a) Čvor cabinet_spawner napravi ormarić, postavi ga u Gazebo i na temu /cabinet/config objavi JSON s opisom.
Spawner objavljuje i TF okvire cabinet_origin (ishodište ormarića) i cabinet_D_initial (točka D na početnom kutu).
Koordinatni sustavi:

W: svijet,
O: ormarić (središte),
A: os zgloba vrata (z_A je os oko koje se vrata okreću),
D: rub krila najudaljeniji od osi, na unutarnjoj plohi krila.

cu.parse_config() pretvori JSON u Python rječnik, a matrice u numpy polja.

b) Odrediti položaj osi rotacije vrata: 
Os je okvir A. Da bi je izrazio u svijetu, treba ti lanac W → O → A:

^W T_A = ^W T_O · ^O T_A

^W T_O dobiješ iz pose: pomak (x, y, z) i zakret yaw oko vertikalne osi:

         ⎡ cos yaw  -sin yaw  0   x ⎤   ⎡-0.148  -0.989  0   0.008⎤
^W T_O = ⎢ sin yaw   cos yaw  0   y ⎥ = ⎢ 0.989  -0.148  0  -0.796⎥
         ⎢ 0         0        1   z ⎥   ⎢ 0       0      1   0.178⎥
         ⎣ 0         0        0   1 ⎦   ⎣ 0       0      0   1    ⎦

^O T_A je gotova u poruci (T_A_cabinet_closed):

         ⎡-1   0  0   0.1415⎤
^O T_A = ⎢ 0  -1  0  -0.155 ⎥
         ⎢ 0   0  1   0     ⎥
         ⎣ 0   0  0   1     ⎦
pomak (0.1415, −0.155, 0): os je na prednjoj strani ormarića (pola dubine minus pola debljine krila) i na rubu otvora (pola širine vrata),
rotacija je zakret za 180° oko z, jer je kod μ = −1 zglob na “drugoj” strani.

Rezultat: ^W T_A = ^W T_O · ^O T_A. Iz te matrice očitaš os:

točka na osi = zadnji stupac (ishodište A): (0.140, −0.633, 0.178),
smjer osi = treći stupac (z_A): (0, 0, 1), dakle os je vertikalna.

c) Izračunati početni položaj sustava S_D u svijetu:
Ovo je jednadžba (2) iz uputa:

^W T_D(φ) = ^W T_O · ^O T_A · Rz(φ) · ^A T_D(0),     φ = μ·θ

Čita se s lijeva na desno: od svijeta do ormarića, od ormarića do osi, zakret vrata oko osi za φ, i od osi do točke D na krilu.

^A T_D(0) računamo sami po jednadžbi (1) iz uputa (funkcija cu.T_A_D0), iz dimenzija krila l = [l_x, l_y, l_z] = [0.017, 0.31, 0.31], r = [0, −l_y/2] i μ = −1:

            ⎡ 0   0  -μ   r_x - μ·l_x/2 ⎤   ⎡ 0  0  1   0.0085⎤
^A T_D(0) = ⎢ μ   0   0   r_y - l_y/2   ⎥ = ⎢-1  0  0  -0.31  ⎥
            ⎢ 0  -1   0   l_z/2         ⎥   ⎢ 0 -1  0   0.155 ⎥
            ⎣ 0   0   0   1             ⎦   ⎣ 0  0  0   1     ⎦

Značenje položaja: D je 0.31 m od osi (na slobodnom rubu krila), na vrhu krila (0.155 = pola visine iznad sredine) i na unutarnjoj plohi (0.0085 = pola debljine).

Rz(φ) je rotacija oko z_A za φ = μ·θ = −1 · 20° = −20°. Kut se množi s μ jer kod μ = −1 okvir A je zakrenut za 180°, pa bi isti predznak zatvarao vrata umjesto da ih otvara.

Rezultat za 20°:

              ⎡-0.980  0  -0.200  -0.165⎤
^W T_D(20°) = ⎢ 0.200  0  -0.980  -0.580⎥
              ⎢ 0     -1   0       0.333⎥
              ⎣ 0      0   0       1    ⎦

Točka D je u svijetu na (−0.165, −0.580, 0.333).

d) Usporediti s T_D_world_initial i TF transformacijom:
Program radi tri usporedbe (funkcija compare):

Usporedba	Što provjerava
jednadžba (1) ↔ T_D_hinge_closed	je li naš ^A T_D(0) isti kao spawnerov
jednadžba (2) ↔ T_D_world_initial	je li naš ^W T_D isti kao spawnerov
jednadžba (2) ↔ TF cabinet_D_initial	slaže li se s okvirom koji spawner objavljuje na TF (preko lookup_transform('world', 'cabinet_D_initial'))

Za svaku usporedbu računaju se dva broja:

razlika položaja |Δp| = udaljenost između dvije točke, u mm,
razlika orijentacije |ΔR| = kut rotacije koja prevodi jednu orijentaciju u drugu, u stupnjevima.

Naredba: ros2 run nrs_lv1_tasks task2_cabinet --ros-args -p use_sim_time:=true
Ispis redom: konfiguracija ormarića, ^W T_O, ^O T_A, os rotacije u svijetu (točka i smjer), ^A T_D(0) s usporedbom, ^W T_D(φ) i na kraju tablica usporedbi s OK.

UKRATKO ZA ZAD 2: 
1. pokreni ROS, stvori čvor (pretplata na /cabinet/config, TF slušač)
2. čekaj do 20 s da stigne konfiguracija
3. izračunaj:  ^W T_O,  ^W T_A (os vrata),  ^A T_D(0) [jedn. 1],  ^W T_D(φ₀) [jedn. 2]
4. usporedi:   jedn. (1) ↔ T_D_hinge_closed
               jedn. (2) ↔ T_D_world_initial
               jedn. (2) ↔ TF cabinet_D_initial
5. ugasi čvor

3) 
