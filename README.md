<div align="center">

  <a href="">[![General badge](https://img.shields.io/badge/LinkedIn-0077B5?style=for-the-badge&logo=linkedin&logoColor=white<SUBJECT>-<STATUS>-<COLOR>.svg)](https://www.linkedin.com/in/luca-lussu-8135a4139/)</a>
  <a href="">[![MIT license](https://img.shields.io/badge/License-MIT-blue.svg)](https://lbesson.mit-license.org/)</a>
 
</div>

# Exercise-for-an-exam-Forward-Kinematics-Inverse-kinematics-Jacobian-
## Introduzione al problema 
Il sitema da analizzare è un manipolatore, ovvero un sistema robotico costituito da un
insieme di braccia connessi da opportuni giunti di movimento. Ad un manipolatore è in
generale collegato un end-effector, cioè l’utensile necessario per effetuare il tipo specifico
di manipolazione (pinza, fresa, ventosa etc.).
Il sistema preso in esame è un manipolatore planare a 2 giunti rotoidali, con due
braccia, l’esempio in figura 1.1 mostra di fatto 2 giunti, 2 braccia e un end-effector, le
variabili di giunto sono gli angoli che ogni braccio forma con il successivo (theta1, theta2).
Ricordiamo che il manipolatore planare a 2 giunti è una catena cinematica aperta, dato
dal fatto che vi è una sola sequenza di bracci che connette gli estremi della catena.

![image](https://user-images.githubusercontent.com/48186637/187909159-3a7ac765-31a9-42ab-a3b3-7816bcb59bd8.png)
> Figura 1.1: Manipolatore planare a due braccia

## Problema proposto
Il problema da affrontare in questo progetto tratta il calcolo della cinematica diretta,
inversa e lo jacobiano del nostro manipolatore planare a due braccia con l’utilizzo del
robotics-toolbox in Python.
In particolare è dato un manipolatore con lunghezza delle braccia L = 30 cm, si deve:
* Calcolare la cinematica diretta del manipolatore per q = [10°, 20°]
* Calcolare la cinematica inversa del manipolatore per raggiungere la poszione pE = [34,
46]cm
* Calcolare lo Jacobiano geometrico del manipolatore

La risoluzione del problema verrà affrontato attraverso l’utilizzo dell’ambiente di
sviluppo PyCharm, all’interno del quale verranno sviluppati n script scritti in Python con
l’ausilio del robotics-toolbox sviluppato da Peter Corck. Tale toolbox contiene degli
strumenti estremamente utili per lo studio della cinematica e della dinamica e la
generazione di traiettorie dei manipolatori, non solo ma contiene anche una cassetta
degli attrezzi molto varia, come ad esempio più di 30 modelli di robot con delle
specifiche già inserite, oppure abbiamo delle notazioni con delle funzioni pronte all’uso
come per la notazione di Denavit-Hartenberg etc.

## Metodologia utilizzata per lo svolgimento
### Calcolo della cinematica diretta

Il problema verrà risolto con due approcci:
* Partendo dalla tabella di Denavit–Hartenberg mi calcolo le relative matrici
omogenee.
* Usando la funzione specifica nel Robotics-Toolbox.

L’equazione della cinematica diretta consente di definire le relazioni funzionali tra le
variabili di giunto e la posa dell’organo terminale, esprime rispetto ad una terna di
riferimento, la posa dell’organo terminale in funzione delle variabili di giunto della
struttura meccanica.
Il primo approccio consiste nell’elaborare il codice riga per riga come se si facesse
con carta e penna, senza usare troppe semplificazioni date dal toolboox, così da poter
verificare in entrambi gli approcci la veridicità dei risultati.
Di seguito verrà utilizzata la convenzione DH per determinare le terne di riferimento e
utilizzare i risultati ottenuti per la determinazione della tabella di DH.

![image](https://user-images.githubusercontent.com/48186637/187910686-54b2ae85-f2f9-4970-ad3f-788d091e9cbf.png)
> Figura 2.1: Manipolatore planare con rispettivi SDR

A questo punto possiamo determinarci la tabella con i relativi parametri. Il codice
inizia con il costrutto RevoluteDH che ci permette di creare un oggetto che reppresenta
ogni riga della tabella di DH.
Successivamente creo una funzione Forward Kinematics che calcola la matrice di
trasformazione omogenea 𝐴𝑖−1
𝑖 (𝑞𝑖), mentre la funzione SE ci permette di definire delle
matrici di trasformazione omogenea e relativi metodi. Tenendo in considerazione come
fatto a esercitazione che i sistemi di riferimento della base è la medesima della terna 0 e
che il sdr dell’end-effector è identica a quella della terna 2, di conseguenza abbiamo due
matrici identità, quindi la trasformata omogenea sarà coincidente con la matrice della
cinematica diretta del manipolatore.

![image](https://user-images.githubusercontent.com/48186637/187910870-9728f2e1-9efd-4e13-b741-f7e199c0ed12.png)
> Figura 2.2: Tabella DH

![image](https://user-images.githubusercontent.com/48186637/187910911-b901b3af-1f9a-44cd-a6e0-9815623a2ed5.png)
> Figura 2.3: codice cinematica diretta DH

Questa funzione sarà invocata per il calcolo delle matrici di trasformazione omogenea
𝐴𝑖−1
𝑖 (𝑞𝑖) per i = 1,..,n.

![image](https://user-images.githubusercontent.com/48186637/187911002-3974a095-9637-43b0-a93c-5de3f4dfb583.png)
> Figura 2.4: codice matrice di trasformazione omogenea

Il secondo approccio riguarda l’utilizzo delle funzioni del robotics-Toolbox, come ad
esempio l’utilizzo della funzione rtb.models.DH.Planar2, che crea immediatamente un
oggetto che rappresenta un manipolatore a due braccia. Grazie alla creazione di questo
oggetto possiamo visualizzare direttamente la tabella di DH e attraverso l’uso di fkine
calcolo direttamente la cinematica diretta del robot in esame.
Si precisa che data l’impossibilità di poter modificare all’interno della tabella di DH la
lunghezza del braccio 𝑎𝑗, è stato possibile modificarlo direttamente dal codice sorgente
Planar2.py, potendo così inserire correttamente le lunghezze delle braccia di 30 cm.

### Calcolo cinematica inversa

Il problema cinematico inverso riguarda la determinazione delle variabili di giunto, una
volta assegnata la posa dell’organo terminale. L’ obiettivo dell’esercizio è calcolare la
cinematica inversa del manipolatore per raggiungere la posizione Px = 34, Py = 46
centimetri. Per risolvere il problema assegnato si è deciso di utilizzare la soluzione
analitica con i seguenti passaggi.

![image](https://user-images.githubusercontent.com/48186637/187911175-dc8b23b1-9c29-4548-9890-03466aec1a77.png)
> Figura 2.5: Codice e verfica condizione workspace

![image](https://user-images.githubusercontent.com/48186637/187911205-cbf42bc2-a94d-4152-b1b2-14ae9881155d.png)
> Figura 2.6: Codice calcolo cinematica inversa

Nella figura 2.6 è stato utilizzato un approccio automatico, data dal
robotics-toolboox, ovvero l’utilizzo della funzione ikine LM che accetta come parametro
la matrice di trasformazione omogenea calcolata con la funzione della cinematica diretta
e mi restituisce le coordinate dei giunti n corrispondenti alla posa finale del robot.


### Calcolo Jacobiano geometrico
Per caratterizzare i legami tra velocità dei giunti e le corrispondenti velocità lineare e
angolare del end-effector, possono essere descritte andando ad utilizzare una matrice di
trasformazione dipendente dalla configurazione del manipolatore, chiamata di fatto
Jacobiano geometrico.
La relazione in fig. 2.5 consente il calcolo dello Jacobiano in maniera sistematica sulla
base di relazioni cinematiche dirette. Infatti i vettori 𝑍𝑖−1, 𝑝𝑒 e 𝑝𝑖−1 risultano funzioni delle
variabili di giunto, in particolare:

![image](https://user-images.githubusercontent.com/48186637/187911331-f0f56987-da5a-4e16-a9d5-e42ba1a084a9.png)
> Figura 2.7: Matrice Jacobiana geometrico

![image](https://user-images.githubusercontent.com/48186637/187911368-2f70fba2-cbec-4684-9e25-6066de52537f.png)
> Figura 2.8: Vettore Zi-1
Quindi per il nostro manipolatore otteniamo quanto segue:
![image](https://user-images.githubusercontent.com/48186637/187911423-f3a7bd42-bbfa-4590-9116-26eba4093677.png)
> Figura 2.9: Calcolo prima colonna Jacobiano

![image](https://user-images.githubusercontent.com/48186637/187911457-c734d1c6-7be9-4481-9077-944250190fbf.png)
> Calcolo seconda colonna Jacobiano

Nei precedenti passaggi abbiamo trovato lo Jacobiano geometrico del nostro
manipolatore a due bracci, ora non ci resta che verificare il risultato con il
robotics-toolboox, dove andremo ad applicare la funzione jacob0 che ci restituisce lo
jacobiano sotto forma di matrice. Nello script scritto in Python utilizziamo tale funzione
sia per verificare la correttezza dello jacobiano calcolato precedentemente, sia per
calcolare lo jacobiano di qualsiasi manipolatore, prendendo come esempio il
manipolatore della Universal Robotcs UR5 con 6 gradi di libertà.

![image](https://user-images.githubusercontent.com/48186637/187911556-42abe441-40ad-46e7-9177-159b0a0b2583.png)
> Figura 2.11: Codice Jacobiano geoemtrico

## Sintesi dei risultati ottenuti

I risultati ottenuti dimostrano la veridicità dei calcoli ottenuti con gli approcci manuali,
in quanto i risultati ottenuti con le funzioni del robotics-toolbox sono i medesimi.

![image](https://user-images.githubusercontent.com/48186637/187911662-5c641355-088a-4218-8f72-445d943c368e.png)
> Figura 3.1: Soluzioni cinematica diretta

![image](https://user-images.githubusercontent.com/48186637/187911698-cf1f3393-3de8-461f-a263-6104995d5f51.png)
> Figura 3.2: Soluzioni cinematica inversa

![image](https://user-images.githubusercontent.com/48186637/187911746-b347075a-abc5-45b4-8105-b41497213a95.png)
> Figura 3.3: Soluzioni Jacobiano geometrico

## Riferimenti 
* Robotica. Modellistica, pianificazione e controllo, di Bruno Siciliano, Lorenzo
Sciavicco, Luigi Villani
* https://petercorke.github.io/robotics-toolbox-python/intro.html#robotics-toolbox







