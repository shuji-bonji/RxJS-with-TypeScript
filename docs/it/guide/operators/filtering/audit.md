---
description: "L'operatore audit è un operatore di filtraggio RxJS che emette solo l'ultimo valore nel periodo controllato dall'Observable personalizzato. È ideale per il controllo dinamico dei tempi."
---

# audit - ultimo valore del periodo di controllo emesso

L'operatore `audit` attende finché un Observable personalizzato non emette un valore ed emette l'**ultimo valore** emesso dalla sorgente entro quel periodo.
Mentre `auditTime` è controllato da un tempo fisso, `audit` permette il **controllo del periodo** con un Observable dinamico.

## 🔰 Sintassi e uso di base

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Evento click
const clicks$ = fromEvent(document, 'click');

// 1Periodi di tempo separati ogni secondo
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('Il clic è stato registrato');
});
```

- Quando si verifica un clic, inizia un periodo di un secondo.
- Viene emesso solo l'ultimo clic di questo periodo di 1 secondo.
- Dopo un secondo, inizia il periodo successivo.

[🌐 Documentazione ufficiale di RxJS - `audit`](https://rxjs.dev/api/operators/audit)

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Evento click
const clicks$ = fromEvent(document, 'click');

// 1Periodi di tempo separati ogni secondo
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('Il clic è stato registrato');
});
```

> L'esempio qui sopra omette la sottoscrizione di `fromEvent` per semplicità di spiegazione. Nel codice reale, si può usare takeUntil(destroy$)`, take(N)` o `Subscription.unsubscribe()` per gestire esplicitamente il ciclo di vita. Ulteriori informazioni: [Superare le difficoltà: la gestione del ciclo di vita] (/it/guide/ Superare le difficoltà/gestione del ciclo di vita.md)

## 💡 Modelli tipici di utilizzo

- **Campionamento dinamico a intervalli**: regolazione della durata in base al carico.
- **Controllo personalizzato dei tempi**: controllo del periodo in base ad altri Observable.
- **Limitazione adattiva degli eventi**: assottigliamento sensibile al contesto.

## 🔍 Differenze con auditTime

| Operatore. | Controllo del periodo | Caso d'uso. |
|---|---|---|
| AuditTime. | Tempo fisso (millisecondi) | Semplice controllo basato sul tempo |
| `audit`. | **Osservable personalizzato**. | **Controllo dinamico del periodo**. |

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, auditTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// auditTime - Fissato1secondi
clicks$.pipe(
  auditTime(1000)
).subscribe(() => console.log('Fissato1secondi'));

// audit - Periodo dinamico
let period = 1000;
clicks$.pipe(
  audit(() => {
    period = Math.random() * 2000; // 0~.2Periodo casuale di secondi
    return timer(period);
  })
).subscribe(() => console.log(`Periodo dinamico: ${period}ms`));
```

## 🧠 Esempio pratico di codice 1: campionamento dinamico basato sul carico

Questo è un esempio di regolazione dell'intervallo di campionamento in base al carico del sistema.

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map } from 'rxjs';

// UICreazione
const output = document.createElement('div');
output.innerHTML = '<h3>Campionamento dinamico</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Modifica del carico';
document.body.appendChild(button);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
output.appendChild(statusDiv);

const logDiv = document.createElement('div');
logDiv.style.marginTop = '10px';
logDiv.style.maxHeight = '200px';
logDiv.style.overflow = 'auto';
output.appendChild(logDiv);

// Livello di carico (0: Carico basso,1: Carico medio,2: carico elevato)
let loadLevel = 0;

fromEvent(button, 'click').subscribe(() => {
  loadLevel = (loadLevel + 1) % 3;
  const levels = ['Carico basso', 'Carico medio', 'Carico elevato'];
  statusDiv.textContent = `Carico corrente: ${levels[loadLevel]}`;
});

// Evento di movimento del mouse
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  audit(() => {
    // La durata dipende dal carico
    const periods = [2000, 1000, 500]; // Carico basso→Lunga durata, carico elevato→Durata breve
    return timer(periods[loadLevel]);
  }),
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe(pos => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Posizione del mouse: (${pos.x}, ${pos.y})`;
  logDiv.insertBefore(log, logDiv.firstChild);

  // Max.10Visualizzazione fino a
  while (logDiv.children.length > 10) {
    logDiv.removeChild(logDiv.lastChild!);
  }
});
```

- Campionamento ridotto a intervalli di 2 s quando il carico è basso (modalità di risparmio energetico).
- Campionamento fine a intervalli di 500 ms quando il carico è elevato.
- Il periodo può essere regolato dinamicamente in base al carico.

## 🎯 Esempio pratico di codice 2: Controllo del periodo basato su altri flussi

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map, startWith } from 'rxjs';

// UICreazione
const container = document.createElement('div');
document.body.appendChild(container);

const slider = document.createElement('input');
slider.type = 'range';
slider.min = '100';
slider.max = '2000';
slider.value = '1000';
container.appendChild(document.createTextNode('Intervallo: '));
container.appendChild(slider);

const intervalDisplay = document.createElement('span');
intervalDisplay.textContent = ' 1000ms';
container.appendChild(intervalDisplay);

const output = document.createElement('div');
output.style.marginTop = '10px';
container.appendChild(output);

// Monitoraggio dei valori del cursore
const sliderValue$ = fromEvent(slider, 'input').pipe(
  map(() => Number(slider.value)),
  startWith(1000)
);

sliderValue$.subscribe(value => {
  intervalDisplay.textContent = ` ${value}ms`;
});

// Evento click
const clicks$ = fromEvent(document, 'click');

let currentInterval = 1000;

// Aggiornare i valori dei cursori
sliderValue$.subscribe(value => {
  currentInterval = value;
});

// Fare clic suauditControllato da
clicks$.pipe(
  audit(() => timer(currentInterval))
).subscribe(() => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Registrazione del clic (intervallo: ${currentInterval}ms)`;
  output.insertBefore(log, output.firstChild);
});
```

## ⚠️ Note.

### 1. il primo valore non viene emesso immediatamente

Dopo aver ricevuto il primo valore, l'audit attende la fine del periodo.

```ts
import { interval, timer } from 'rxjs';
import { audit, take } from 'rxjs';

interval(100).pipe(
  audit(() => timer(1000)),
  take(3)
).subscribe(val => {
  console.log(val);
});
// Uscita:
// 9  (1Secondi dopo,0~.9Ultimo valore di)
// 19 (2Secondi dopo,10~.19Ultimo valore di)
// 29 (3Secondi dopo,20~.29Ultimo valore di)
```

### 2. L'Observable della durata viene generato ogni volta.

Le funzioni passate ad audit **devono restituire ogni volta un nuovo Observable**.

```ts
// ❌ Esempio negativo: Se la stessaObservableviene utilizzata e riutilizzata
const duration$ = timer(1000);
source$.pipe(
  audit(() => duration$) // 2Non funziona dopo la seconda volta
).subscribe();

// ✅ Buon esempio: Una nuovaObservableGenera un
source$.pipe(
  audit(() => timer(1000))
).subscribe();
```

### 3. memoria e prestazioni

L'uso di audit sui flussi in cui vengono emessi valori consuma spesso memoria.

```ts
import { interval, timer } from 'rxjs';
import { audit } from 'rxjs';

// flusso veloce (10msal secondo)
interval(10).pipe(
  audit(() => timer(1000)) // 1Campionamento ogni secondo
).subscribe();
// 1al secondo100I valori vengono memorizzati e solo l'ultimo viene emesso.1Viene emesso solo l'ultimo
```

## 🆚 Confronto con operatori simili

{__TABELLA_10___

```ts
import { fromEvent, interval, timer } from 'rxjs';
import { audit, throttle, debounce, sample } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// audit: 1Ultimo clic in secondi
clicks$.pipe(
  audit(() => timer(1000))
).subscribe(() => console.log('audit: Ultimo clic'));

// throttle: 1Primo clic in secondi
clicks$.pipe(
  throttle(() => timer(1000))
).subscribe(() => console.log('throttle: Primo'));

// debounce: Dopo l'arresto del clic1Secondi dopo
clicks$.pipe(
  debounce(() => timer(1000))
).subscribe(() => console.log('debounce: Dopo l'arresto'));

// sample: 1Campionamento ogni secondo
clicks$.pipe(
  sample(interval(1000))
).subscribe(() => console.log('sample: Periodico'));
```

## 📚 Operatori correlati.

- **[auditTime](./auditTime)** - controllato da un tempo fisso (versione semplificata di `audit`).
- **[throttle](./throttleTime)** - primo valore emesso all'inizio del periodo.
- **[debounce](./debounceTime)** - emette un valore dopo un periodo di inattività.
- **[sample](./sampleTime)** - campiona al momento di un altro Observable.

## Riepilogo.

L'operatore audit emette l'ultimo valore all'interno di un periodo controllato dinamicamente da un Observable personalizzato.

- È possibile un controllo dinamico del periodo.
- ✅ Campionamento adattivo basato sul carico.
- ✅ Controllo basato su altri flussi
- ⚠️ È necessario generare ogni volta un nuovo Observable.
- ⚠️ Sensibile alla memoria per emissioni frequenti
