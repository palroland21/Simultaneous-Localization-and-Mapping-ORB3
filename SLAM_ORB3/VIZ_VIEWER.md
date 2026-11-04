# Viewer OpenCV Viz

Fereastra 3D folosește `cv::viz::Viz3d` (VTK), fără panou lateral. Fundalul este alb; punctele normale sunt negre, referințele roșii, camera curentă verde, cadrele-cheie active albastre, iar rădăcinile roșii cu linii mai groase. Hărțile inactive păstrează paleta originală. Dimensiunile provin din parametrii `Viewer.*` existenți.

Ferestrele imaginilor, concatenarea stereo, redimensionarea și suprapunerile personalizate sunt păstrate prin `FrameDrawer`. Graful de covizibilitate, arborele, buclele și legăturile inerțiale sunt afișate în continuare. Exportul traiectoriilor și evaluarea rămân în aplicație.

## Dependențe

În acest mediu sunt folosite OpenCV **4.10.0**, selectat prin `/usr/lib/x86_64-linux-gnu/cmake/opencv4`, și VTK **9.5.2**. Modulul contrib `viz` și antetele VTK sunt necesare. Pachetele sunt deja instalate; pentru o instalare Ubuntu echivalentă:

```bash
sudo apt install libopencv-dev libopencv-contrib-dev libopencv-viz-dev libvtk9-dev
```

CMake cere explicit `viz` din aceeași instalare OpenCV ca restul aplicației. Un wheel Python nu înlocuiește bibliotecile C++ necesare. Pentru o compilare OpenCV proprie, folosește aceeași versiune pentru OpenCV și contrib, cu `-DOPENCV_EXTRA_MODULES_PATH=/cale/opencv_contrib/modules -DWITH_VTK=ON -DBUILD_opencv_viz=ON`, apoi configurează proiectul cu `-DOpenCV_DIR=/prefix/lib/cmake/opencv4`.

## Compilare și rulare

Din rădăcina `ORB3_PROJECT`, cu dependențele terțe deja construite:

```bash
cmake -S SLAM_ORB3 -B SLAM_ORB3/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
cmake --build SLAM_ORB3/build --target mono_euroc stereo_euroc mono_inertial_euroc stereo_inertial_euroc --parallel 2
./orb3
```

Este suficient și `./orb3` singur: scriptul construiește incremental cele patru executabile, apoi afișează meniurile:

```text
How would you like to run ORB-SLAM3?
  1. Monocular (one camera)
  2. Stereo (two cameras)
  3. Monocular + IMU
  4. Stereo + IMU
  0. Exit

Which dataset would you like to use?
  1. MH_01_easy
  2. V1_01_easy
  ...
  0. Exit
```

Modurile Stereo și Stereo + IMU afișează imaginile cam0 (stânga) și cam1 (dreapta) alăturat, inclusiv marcajele punctelor urmărite. Modurile Monocular folosesc și afișează doar cam0. Monocular + IMU și Stereo + IMU folosesc aceeași implementare pentru liniile `ORB3 LIVE`, `ORB3 EVAL` și `ORB3 FINAL`, cu aceleași câmpuri, precizie și ordine. În Monocular + IMU, `CAM1=0.0Hz` indică faptul că a doua cameră nu este utilizată. Poziția și orientarea sunt afișate după inițializarea IMU; până atunci apar `Position=N/A` și motivul așteptării. Evaluarea folosește același ground truth al corpului/IMU, aceeași aliniere rigidă fără ajustarea scării și aceeași variabilă `ORB3_GT`.

În terminal, în fiecare meniu se introduce numărul și se confirmă cu Enter. Cifra tastată rămâne vizibilă, iar opțiunea aleasă este afișată înainte de continuare. `0` sau `q` închide meniul. Dataseturile extrase sunt filtrate după camerele și IMU necesare; fișierele ROS1 `.bag` pot fi extrase la alegere. Se aleg automat executabilul și calibrarea EuRoC ale modului. Modurile RGB-D necesită dataseturi cu adâncime, deci nu sunt oferite în acest launcher EuRoC.

Comanda directă `./orb3 MH_01_easy` păstrează modul Stereo + IMU. Pentru celelalte moduri, rezultatele sunt salvate separat în `outputs/<dataset>/<executabil>/`; Stereo + IMU păstrează `outputs/<dataset>/`. Monocular folosește timestampurile camerei stângi, iar Stereo folosește perechile sincronizate.

`ORB3_DATASETS` permite selectarea directorului dataseturilor. `ORB3_OPENCV_DIR` permite selectarea altei instalări OpenCV; `ORB3_VIEWER=0 ./orb3` dezactivează ferestrele în toate cele patru moduri.

Terminalul VS Code instalat prin Snap poate moșteni variabile GTK care încarcă biblioteci incompatibile. Dacă apare o eroare GLIBC din bibliotecile Snap, folosește un terminal obișnuit sau:

```bash
env -u GTK_PATH -u GTK_EXE_PREFIX -u GTK_IM_MODULE_FILE -u GIO_MODULE_DIR -u GDK_PIXBUF_MODULEDIR -u GDK_PIXBUF_MODULE_FILE QT_QPA_PLATFORM=xcb QT_QPA_PLATFORMTHEME=generic QT_STYLE_OVERRIDE=Fusion ./orb3
```

## Comenzi

- Stânga + mișcare: deplasare în planul imaginii, la adâncimea selectată sub cursor.
- Dreapta + mișcare: orbită în jurul punctului selectat, schimbând poziția și orientarea perspectivei.
- Mijloc + mișcare: rotirea direcției de privire din poziția curentă.
- Stânga și dreapta simultan + mișcare orizontală: rotire în planul imaginii.
- Rotiță: apropiere/depărtare față de punctul selectat.
- `T` sau clic pe textul albastru din stânga-jos: activează/dezactivează vederea de sus care urmărește camera. `[ON]` indică urmărirea activă.
- Interacțiunea cu mouse-ul redă controlul manual; `T` reia urmărirea.
- `R`: perspectiva inițială, fără resetarea hărții sau a SLAM.

`T` și `R` funcționează și când fereastra imaginilor este focalizată. Închiderea uneia dintre ferestrele viewerului le închide pe amândouă; procesarea datasetului continuă. La final, `System::Shutdown()` solicită oprirea și face `join` pe firul viewerului.

## Implementare

`VizMouseNavigation` păstrează matematica navigării originale, cu sensibilitate de rotație `0.01` radiani/pixel, selecția adâncimii într-o zonă de 17×17 pixeli și aceeași tratare a fundalului și combinațiilor de butoane. Codul include nota de copyright și licența MIT a autorului original.

Proiecția și planele `0.1`/`1000` sunt fixate explicit în VTK. TOP folosește axa Z verticală după inițializarea IMU și convenția inițială -Y în sus înainte de inițializare sau pentru hărțile fără IMU. Urmărirea modifică numai perspectiva și este activată explicit.

`ProcessVizEvents()` inițializează interactorul o dată, apoi folosește `ProcessEvents()` și randarea ferestrei. Astfel, bucla continuă să actualizeze imaginile și când un buton rămâne apăsat. HighGUI procesează evenimente prin `cv::waitKey()`.

`MapDrawer::UpdateViewer()` copiază datele prin accesările sincronizate existente. Achiziția mutexului hărții folosește `try_lock`; dacă harta este ocupată, scena existentă este păstrată și camera actualizată, fără a bloca afișarea imaginilor. Operațiile VTK au loc după eliberarea blocărilor SLAM. Resetările opresc viewerul înainte de golirea hărții.

Widgeturile au identificatori stabili. Graful folosește date VTK cu toate segmentele pentru fiecare tip de legătură și actori persistenți. Aceasta evită reconstruirea unui pipeline pentru fiecare muchie.

Selecția punctelor păstrează regulile originale: punctele valide obișnuite ale hărții active sunt negre, iar setul unic de referințe valide este roșu, inclusiv referințele temporar în afara listei hărții. Pozițiile nefinite sunt omise. Viewerul nu generează și nu dublează puncte. `Viewer.PointSize` este neschimbat.

Pozițiile cadrelor-cheie sunt `GetPoseInverse()`, iar camera curentă inversează o singură dată `Tcw` în `SetCurrentCameraPose()`. Adaptorul de navigare modifică numai camera VTK, fără a modifica trackingul, optimizarea sau calibrarea.

Profilarea rămâne opțională: `ORB3_VIEWER_PROFILE=1 ./orb3 MH_01_easy` afișează sumarul timpilor la ieșire. Backendul Qt poate emite avertismente despre timere la închidere, asociate ferestrelor HighGUI din firul viewerului.
