---
description: windowToggle è un operatore di conversione RxJS avanzato che permette di gestire più periodi di finestra indipendentemente, con trigger di inizio e fine controllati da Observable separati. È ideale per situazioni dove è richiesta una gestione dinamica dei periodi, come la raccolta dati durante l'orario lavorativo o la registrazione eventi durante la pressione dei bottoni. L'inferenza dei tipi di TypeScript garantisce un'elaborazione di divisione finestre type-safe.
titleTemplate: ':title | RxJS'
---

# windowToggle - Finestra con Controllo Indipendente di Inizio e Fine

L'operatore `windowToggle` controlla il **trigger di inizio** e il **trigger di fine** con Observable separati, emettendo ogni periodo come un nuovo Observable. Questo è un operatore finestra avanzato che può gestire più periodi di finestra simultaneamente.

## 🔰 Sintassi e Utilizzo Base

```ts
import { interval } from 'rxjs';
import { windowToggle, mergeAll } from 'rxjs';

const source$ = interval(500); // Emetti valori ogni 0.5 secondi

// Trigger di inizio: ogni 2 secondi
const opening$ = interval(2000);

// Trigger di fine: 1 secondo dopo l'inizio
const closing = () => interval(1000);

source$.pipe(
  windowToggle(opening$, closing),
  mergeAll()
).subscribe(value => {
  console.log('Valore nella finestra:', value);
});

// Inizia a 2 sec, finisce a 3 sec → Valori: 4, 5
// Inizia a 4 sec, finisce a 5 sec → Valori: 8, 9
// Inizia a 6 sec, finisce a 7 sec → Valori: 12, 13
```

**Flusso di operazione**:
1. `opening$` emette un valore → La finestra inizia
2. L'Observable restituito da `closing()` emette un valore → La finestra finisce
3. Più periodi di finestra possono sovrapporsi

[🌐 Documentazione Ufficiale RxJS - `windowToggle`](https://rxjs.dev/api/operators/windowToggle)

## 💡 Pattern di Utilizzo Tipici

- Raccolta dati durante l'orario lavorativo
- Registrazione eventi durante la pressione di bottoni
- Tracciamento azioni durante sessioni attive
- Elaborazione stream che richiede gestione dinamica dei periodi

## 🔍 Differenza da bufferToggle

| Operatore | Output | Caso d'Uso |
|:---|:---|:---|
| `bufferToggle` | **Array (T[])** | Elabora insieme i valori raggruppati |
| `windowToggle` | **Observable&lt;T&gt;** | Elaborazione stream diversa per ogni gruppo |

```ts
import { interval } from 'rxjs';
import { bufferToggle, windowToggle, mergeAll } from 'rxjs';

const source$ = interval(500);
const opening$ = interval(2000);
const closing = () => interval(1000);

// bufferToggle - Output come array
source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Output: Buffer (array): [4, 5]
});

// windowToggle - Output come Observable
source$.pipe(
  windowToggle(opening$, closing)
).subscribe(window$ => {
  console.log('Window (Observable):', window$);
  window$.subscribe(value => {
    console.log('  Valore nella finestra:', value);
  });
});
```

## 🧠 Esempio di Codice Pratico 1: Registrazione Eventi Durante Pressione Bottone

Questo è un esempio di registrazione dati tra mouse down e mouse up.

```ts
import { fromEvent, interval } from 'rxjs';
import { windowToggle, mergeMap, toArray, take } from 'rxjs';

// Crea bottone
const button = document.createElement('button');
button.textContent = 'Tieni premuto';
document.body.appendChild(button);

// Area di output
const display = document.createElement('div');
display.style.marginTop = '10px';
document.body.appendChild(display);

// Stream dati (ogni 100ms)
const data$ = interval(100);

// Inizio: Mouse giù
const mouseDown$ = fromEvent(button, 'mousedown');

// Fine: Mouse su
const mouseUp = () => fromEvent(document, 'mouseup').pipe(take(1));

data$.pipe(
  windowToggle(mouseDown$, mouseUp),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(events => {
  display.textContent = `Eventi registrati durante la pressione: ${events.length} elementi`;
  console.log('Dati registrati:', events);
});
```

## 🎯 Esempio di Codice Pratico 2: Raccolta Dati Durante Orario Lavorativo

Questo è un esempio di raccolta dati sensore dall'inizio dell'orario lavorativo alla fine.

```ts
import { interval, timer } from 'rxjs';
import { windowToggle, mergeMap, scan, map } from 'rxjs';

// Dati sensore (sempre in acquisizione)
const sensorData$ = interval(100).pipe(
  map(() => ({
    timestamp: Date.now(),
    temperature: 20 + Math.random() * 10, // 20-30 gradi
    humidity: 40 + Math.random() * 20     // 40-60%
  }))
);

// Apertura attività: dopo 2 secondi, poi ogni 10 secondi
const businessOpen$ = timer(2000, 10000);

// Chiusura attività: 5 secondi dopo l'inizio
const businessClose = () => timer(5000);

let sessionNumber = 0;

sensorData$.pipe(
  windowToggle(businessOpen$, businessClose),
  mergeMap(window$ => {
    const current = ++sessionNumber;
    console.log(`Sessione lavorativa ${current} iniziata`);

    // Calcola statistiche per ogni finestra
    return window$.pipe(
      scan((stats, data) => ({
        count: stats.count + 1,
        totalTemp: stats.totalTemp + data.temperature,
        totalHumidity: stats.totalHumidity + data.humidity
      }), { count: 0, totalTemp: 0, totalHumidity: 0 }),
      map(stats => ({
        session: current,
        count: stats.count,
        avgTemp: stats.totalTemp / stats.count,
        avgHumidity: stats.totalHumidity / stats.count
      }))
    );
  })
).subscribe(stats => {
  console.log(`Sessione ${stats.session}: ${stats.count} campioni`);
  console.log(`  Temperatura media: ${stats.avgTemp.toFixed(1)}°C`);
  console.log(`  Umidità media: ${stats.avgHumidity.toFixed(1)}%`);
});
```

## 🎯 Esempio Pratico: Gestione Periodo Download

Questo è un esempio di gestione dei periodi di download dati con bottoni di avvio e stop.

```ts
import { interval, fromEvent, Subject } from 'rxjs';
import { windowToggle, mergeMap, toArray, map } from 'rxjs';

// Crea elementi UI
const startButton = document.createElement('button');
startButton.textContent = 'Avvia';
document.body.appendChild(startButton);

const stopButton = document.createElement('button');
stopButton.textContent = 'Stop';
stopButton.disabled = true;
stopButton.style.marginLeft = '10px';
document.body.appendChild(stopButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = 'In attesa...';
document.body.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
document.body.appendChild(result);

// Stream dati (genera dati di download ogni 1 secondo)
const downloadData$ = interval(1000).pipe(
  map(i => ({
    id: i,
    size: Math.floor(Math.random() * 1000) + 100, // 100-1100KB
    timestamp: new Date()
  }))
);

// Trigger di avvio e stop
const start$ = fromEvent(startButton, 'click');
const stop$ = new Subject<void>();

fromEvent(stopButton, 'click').subscribe(() => {
  stop$.next();
  status.textContent = 'Fermato';
  startButton.disabled = false;
  stopButton.disabled = true;
});

start$.subscribe(() => {
  status.textContent = 'Download in corso...';
  startButton.disabled = true;
  stopButton.disabled = false;
});

// Gestione finestre
downloadData$.pipe(
  windowToggle(start$, () => stop$),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(downloads => {
  const totalSize = downloads.reduce((sum, d) => sum + d.size, 0);
  const avgSize = downloads.length > 0 ? totalSize / downloads.length : 0;

  result.innerHTML = `
    <strong>Download Completato</strong><br>
    Conteggio: ${downloads.length} elementi<br>
    Dimensione totale: ${(totalSize / 1024).toFixed(2)} MB<br>
    Dimensione media: ${avgSize.toFixed(0)} KB
  `;
});
```

## 🎯 Periodi di Finestra Sovrapposti

Una caratteristica di `windowToggle` è che può gestire più periodi di finestra simultaneamente.

```ts
import { interval } from 'rxjs';
import { windowToggle, mergeMap, toArray, take } from 'rxjs';

const source$ = interval(200).pipe(take(20)); // 0-19

// Inizio: ogni 1 secondo
const opening$ = interval(1000);

// Fine: 1.5 secondi dopo l'inizio
const closing = () => interval(1500);

source$.pipe(
  windowToggle(opening$, closing),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Finestra:', values);
});

// Output:
// Finestra: [4, 5, 6, 7]       (Inizia a 1 sec → Finisce a 2.5 sec)
// Finestra: [9, 10, 11, 12]    (Inizia a 2 sec → Finisce a 3.5 sec)
// Finestra: [14, 15, 16, 17]   (Inizia a 3 sec → Finisce a 4.5 sec)
```

**Timeline**:
```
Sorgente:  0--1--2--3--4--5--6--7--8--9--10-11-12-13-14-15-16-17-18-19
Inizio:    ----1sec----2sec----3sec----4sec
Periodo 1: [------1.5sec-----]
            └→ Finestra 1: [4,5,6,7]
Periodo 2:        [------1.5sec-----]
                   └→ Finestra 2: [9,10,11,12]
Periodo 3:               [------1.5sec-----]
                          └→ Finestra 3: [14,15,16,17]
```

## ⚠️ Note

### 1. Gestione Subscription delle Finestre

Ogni finestra è un Observable indipendente, quindi devi sottoscriverlo esplicitamente o appiattirlo con `mergeAll()` o simili.

```ts
source$.pipe(
  windowToggle(opening$, closing)
).subscribe(window$ => {
  // I valori non fluiranno a meno che non ti iscrivi alla finestra stessa
  window$.subscribe(value => {
    console.log('Valore:', value);
  });
});
```

### 2. Attenzione ai Memory Leak

Se i trigger di inizio sono troppo frequenti, molte finestre esisteranno allo stesso tempo, consumando memoria.

```ts
// ❌ Esempio sbagliato: Inizio ogni 100ms, fine dopo 5 secondi
const opening$ = interval(100); // Troppo frequente
const closing = () => interval(5000);

source$.pipe(
  windowToggle(opening$, closing)
).subscribe();
// Fino a 50 finestre possono esistere simultaneamente → Rischio memoria

// ✅ Esempio corretto: Imposta un intervallo appropriato
const opening$ = interval(2000); // Ogni 2 secondi
const closing = () => interval(1000); // Per 1 secondo
```

### 3. Periodi di Finestra Sovrapposti

I periodi di finestra sovrapposti risultano nello stesso valore contenuto in più finestre. Verifica se questo è il comportamento desiderato.

```ts
// Con overlap
opening$ = interval(1000);    // Inizio ogni 1 secondo
closing = () => interval(1500); // Per 1.5 secondi

// Senza overlap
opening$ = interval(2000);    // Inizio ogni 2 secondi
closing = () => interval(1000); // Per 1 secondo
```

## 🆚 Confronto degli Operatori window

| Operatore | Controllo | Periodo Finestra | Caso d'Uso |
|:---|:---|:---|:---|
| `window` | Un altro Observable emette | Continuo | Partizionamento guidato da eventi |
| `windowTime` | Intervallo di tempo fisso | Continuo | Partizionamento basato sul tempo |
| `windowCount` | Conteggio fisso | Continuo | Partizionamento basato sul conteggio |
| `windowToggle` | **Controllo separato inizio/fine** | **Può sovrapporsi** | **Condizioni complesse inizio/fine** |
| `windowWhen` | Solo controllo fine | Continuo | Controllo periodico semplice |

## 🔄 Differenza da windowWhen

```ts
import { interval, timer } from 'rxjs';
import { windowToggle, windowWhen, mergeAll } from 'rxjs';

const source$ = interval(200);

// windowToggle: Controllo separato di inizio e fine
source$.pipe(
  windowToggle(
    interval(1000),          // Trigger di inizio
    () => timer(500)         // Trigger di fine (500ms dopo l'inizio)
  ),
  mergeAll()
).subscribe();

// windowWhen: Controlla solo il timing di fine (il prossimo inizia subito dopo la fine)
source$.pipe(
  windowWhen(() => timer(1000)), // Finestra ogni 1 secondo
  mergeAll()
).subscribe();
```

| Operatore | Controllo | Periodo Finestra | Caso d'Uso |
|:---|:---|:---|:---|
| `windowToggle(open$, close)` | Controllo separato inizio/fine | Può sovrapporsi | Condizioni complesse inizio/fine |
| `windowWhen(closing)` | Solo controllo fine | Continuo | Finestra periodica semplice |

## 📚 Operatori Correlati

- [bufferToggle](/it/guide/operators/transformation/bufferToggle) - Raccogli valori come array (versione array di windowToggle)
- [window](/it/guide/operators/transformation/window) - Dividi finestra a timing di Observable diversi
- [windowTime](/it/guide/operators/transformation/windowTime) - Divisione finestre basata sul tempo
- [windowCount](/it/guide/operators/transformation/windowCount) - Divisione finestre basata sul conteggio
- [windowWhen](/it/guide/operators/transformation/windowWhen) - Divisione finestre con condizioni di chiusura dinamiche

## Riepilogo

L'operatore `windowToggle` è uno strumento avanzato che ti permette di controllare inizio e fine indipendentemente e trattare ogni periodo come un Observable indipendente.

- ✅ Inizio e fine possono essere controllati separatamente
- ✅ Più finestre possono essere gestite simultaneamente
- ✅ Elaborazione diversa può essere applicata a ogni finestra
- ⚠️ Richiede gestione subscription
- ⚠️ Trigger di inizio frequenti consumano memoria
- ⚠️ Fai attenzione ai periodi di finestra sovrapposti
