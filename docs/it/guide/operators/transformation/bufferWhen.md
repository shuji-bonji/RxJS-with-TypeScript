---
description: bufferWhen è un operatore di conversione RxJS che controlla dinamicamente la condizione di fine ed emette i valori in un array. Abilita il buffering continuo dove il buffer successivo inizia immediatamente dopo la fine del buffer, e può essere usato per aggregazione dati flessibile basata sul carico, come elaborazione batch adattiva e raccolta log. L'inferenza dei tipi di TypeScript abilita il buffering dinamico type-safe.
---

# bufferWhen - Buffer con Controllo Dinamico della Fine

L'operatore `bufferWhen` emette un array di valori con **condizioni di fine controllate dinamicamente**. Fornisce un pattern di buffering continuo dove un buffer finisce e il buffer successivo inizia immediatamente.

## 🔰 Sintassi e Utilizzo Base

```ts
import { interval } from 'rxjs';
import { bufferWhen, take } from 'rxjs';

const source$ = interval(500); // Emetti valori ogni 0.5 secondi

// Condizione di fine: dopo 1 secondo
const closingSelector = () => interval(1000);

source$.pipe(
  bufferWhen(closingSelector),
  take(4)
).subscribe(console.log);
// Output:
// [0]           (Inizia a 0 sec → Finisce a 1 sec, solo valore 0)
// [1, 2, 3]     (Inizia a 1 sec → Finisce a 2 sec, valori 1,2,3)
// [4, 5]        (Inizia a 2 sec → Finisce a 3 sec, valori 4,5)
// [6, 7]        (Inizia a 3 sec → Finisce a 4 sec, valori 6,7)
```

**Flusso di operazione**:
1. Il primo buffer inizia automaticamente
2. L'Observable restituito da `closingSelector()` emette un valore → Il buffer finisce, emette array
3. **Il buffer successivo inizia immediatamente** (spesso nello stesso momento dell'emissione di source$)
4. Ripeti 2-3

> [!NOTE]
> Il primo buffer contiene solo `[0]` perché è il periodo di 1 secondo fino a quando `interval(1000)` emette il suo primo valore. Dal secondo buffer in poi, l'inizio del buffer e l'emissione di `source$` coincidono, quindi contengono più valori.

[🌐 Documentazione Ufficiale RxJS - `bufferWhen`](https://rxjs.dev/api/operators/bufferWhen)

## 🆚 Differenza da bufferToggle

`bufferWhen` e `bufferToggle` sono simili, ma **i loro metodi di controllo e pattern di comportamento sono molto diversi**.

### Comportamento di bufferWhen

```ts
import { interval } from 'rxjs';
import { bufferWhen, take } from 'rxjs';

const source$ = interval(300).pipe(take(12)); // Emetti 0-11 ogni 300ms

// bufferWhen: Controlla solo la fine (il prossimo inizia subito dopo la fine)
source$.pipe(
  bufferWhen(() => interval(1000))
).subscribe(console.log);
// Output: [0, 1, 2], [3, 4, 5], [6, 7, 8, 9], [10, 11]
//
// Timeline:
//  0ms   300ms  600ms  900ms  1200ms 1500ms 1800ms 2100ms 2400ms 2700ms 3000ms 3300ms 3600ms
//  0     1      2      3      4      5      6      7      8      9      10     11
//  [----------1sec----------][----------1sec----------][----------1sec----------][-----1sec-----]
//   Buffer1(0-2)              Buffer2(3-5)              Buffer3(6-9)             Buffer4(10-11)
//   Continuo, senza overlap, il prossimo inizia immediatamente
```

### Comportamento di bufferToggle

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(12)); // Emetti 0-11 ogni 300ms

// bufferToggle: Controllo separato di inizio e fine (può sovrapporsi)
const opening$ = interval(1000); // Inizio ogni 1 secondo
const closing = () => interval(800); // Fine 800ms dopo l'inizio

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Output: [3, 4, 5], [6, 7, 8], [9, 10, 11]
//
// Timeline:
//  0ms   300ms  600ms  900ms  1200ms 1500ms 1800ms 2100ms 2400ms 2700ms 3000ms 3300ms
//  0     1      2      3      4      5      6      7      8      9      10     11
//        ----Inizio1(1000ms)----[---Fine dopo 800ms(1800ms)---]
//                        3      4      5
//                        └→ [3,4,5]
//                    ----Inizio2(2000ms)----[---Fine dopo 800ms(2800ms)---]
//                                            6      7      8
//                                            └→ [6,7,8]
//                              ----Inizio3(3000ms)----[---Fine dopo 800ms(3800ms)---]
//                                                      9      10     11
//                                                      └→ [9,10,11]
//  Attende il trigger di inizio, i periodi sono indipendenti (0-2 prima dell'inizio del buffer non inclusi)
```

### Principali Differenze

| Operatore | Controllo Inizio | Controllo Fine | Periodo Buffer | Caratteristica |
|---|---|---|---|---|
| `bufferWhen(closing)` | Auto (subito dopo la fine) | Dinamico | Continuo | Nessun gap tra i buffer |
| `bufferToggle(open$, close)` | Observable indipendente | Dinamico | Indipendente, può sovrapporsi | Gap tra i buffer |

**Linee guida di utilizzo**:
- **`bufferWhen`**: Bufferizza tutti i dati continuamente senza omissioni (logging, aggregazione dati, ecc.)
- **`bufferToggle`**: Raccogli dati solo durante periodi specifici (durante orario lavorativo, pressioni bottone, ecc.)

> [!TIP]
> - **Buffering continuo** (senza perdita dati) → `bufferWhen`
> - **Buffering a periodo limitato** (controllo esplicito inizio/fine) → `bufferToggle`

## 💡 Pattern di Utilizzo Tipici

1. **Raccolta Dati a Intervalli di Tempo Dinamici**
   ```ts
   import { interval, timer } from 'rxjs';
   import { bufferWhen, map } from 'rxjs';

   // Dati sensore
   const sensorData$ = interval(100).pipe(
     map(() => ({
       timestamp: Date.now(),
       temperature: 20 + Math.random() * 10
     }))
   );

   // Condizione di fine: Cambia dinamicamente in base alla temperatura precedente
   let previousAvgTemp = 25;

   sensorData$.pipe(
     bufferWhen(() => {
       // Temperatura più alta = intervallo buffer più corto
       const duration = previousAvgTemp > 27 ? 500 : 1000;
       return timer(duration);
     })
   ).subscribe(data => {
     const avgTemp = data.reduce((sum, d) => sum + d.temperature, 0) / data.length;
     previousAvgTemp = avgTemp;
     console.log(`Temp media: ${avgTemp.toFixed(1)}°C, Campioni: ${data.length}`);
   });
   ```

2. **Elaborazione Batch Adattiva Basata sul Carico**
   ```ts
   import { fromEvent, timer } from 'rxjs';
   import { bufferWhen, map } from 'rxjs';

   interface Task {
     id: number;
     timestamp: number;
   }

   // Stream task
   let taskCounter = 0;
   const tasks$ = fromEvent(document, 'click').pipe(
     map(() => ({
       id: taskCounter++,
       timestamp: Date.now()
     } as Task))
   );

   // Regola il periodo del buffer successivo in base alla dimensione del buffer
   tasks$.pipe(
     bufferWhen(() => timer(2000))
   ).subscribe(bufferedTasks => {
     if (bufferedTasks.length > 0) {
       console.log(`Elaborazione batch: ${bufferedTasks.length} task`);
       console.log('ID Task:', bufferedTasks.map(t => t.id));

       // Determina dinamicamente il periodo del buffer successivo
       // (In pratica, sposta questa logica dentro la funzione bufferWhen)
     }
   });
   ```

3. **Campionamento a Intervalli Casuali**
   ```ts
   import { interval, timer } from 'rxjs';
   import { bufferWhen, map } from 'rxjs';

   // Stream dati
   const data$ = interval(100).pipe(
     map(i => ({
       value: Math.sin(i / 10) * 100,
       timestamp: Date.now()
     }))
   );

   // Buffer a intervalli casuali (500ms ~ 2000ms)
   data$.pipe(
     bufferWhen(() => {
       const randomDelay = 500 + Math.random() * 1500;
       return timer(randomDelay);
     })
   ).subscribe(samples => {
     const avg = samples.reduce((sum, s) => sum + s.value, 0) / samples.length;
     console.log(`Conteggio campioni: ${samples.length}, Media: ${avg.toFixed(2)}`);
   });
   ```

## 🧠 Esempio di Codice Pratico (Raccolta Log Basata sul Carico)

Questo è un esempio di cambiamento dinamico della frequenza di raccolta log in base al carico del sistema.

```ts
import { interval, timer, fromEvent } from 'rxjs';
import { bufferWhen, map, share } from 'rxjs';

// Crea elementi UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Sistema di Raccolta Log Adattivo';
container.appendChild(title);

const loadButton = document.createElement('button');
loadButton.textContent = 'Genera Carico';
container.appendChild(loadButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.style.padding = '10px';
status.style.backgroundColor = '#f0f0f0';
status.textContent = 'Carico basso: Raccogli a intervalli di 5 secondi';
container.appendChild(status);

const logDisplay = document.createElement('pre');
logDisplay.style.marginTop = '10px';
logDisplay.style.padding = '10px';
logDisplay.style.backgroundColor = '#f9f9f9';
logDisplay.style.maxHeight = '300px';
logDisplay.style.overflow = 'auto';
container.appendChild(logDisplay);

// Stream log (sempre in generazione)
let logCounter = 0;
const logs$ = interval(200).pipe(
  map(() => ({
    id: logCounter++,
    level: Math.random() > 0.7 ? 'ERROR' : 'INFO',
    message: `Messaggio log ${logCounter}`,
    timestamp: new Date()
  })),
  share()
);

// Contatore carico (incrementa al click del bottone)
let loadLevel = 0;
fromEvent(loadButton, 'click').subscribe(() => {
  loadLevel = Math.min(loadLevel + 1, 5);
  updateStatus();
});

// Diminuisci carico ogni 30 secondi
interval(30000).subscribe(() => {
  loadLevel = Math.max(loadLevel - 1, 0);
  updateStatus();
});

function updateStatus() {
  const interval = getBufferInterval(loadLevel);
  const loadText = loadLevel === 0 ? 'Carico basso' :
                   loadLevel <= 2 ? 'Carico medio' : 'Carico alto';
  status.textContent = `${loadText} (Livello ${loadLevel}): Raccogli a intervalli di ${interval / 1000} secondi`;
  status.style.backgroundColor =
    loadLevel === 0 ? '#d4edda' :
    loadLevel <= 2 ? '#fff3cd' : '#f8d7da';
}

function getBufferInterval(load: number): number {
  // Carico più alto = intervallo buffer più corto
  switch (load) {
    case 0: return 5000;  // 5 secondi
    case 1: return 3000;  // 3 secondi
    case 2: return 2000;  // 2 secondi
    case 3: return 1000;  // 1 secondo
    case 4: return 500;   // 0.5 secondi
    default: return 300;  // 0.3 secondi
  }
}

// Buffering adattivo
logs$.pipe(
  bufferWhen(() => timer(getBufferInterval(loadLevel)))
).subscribe(bufferedLogs => {
  if (bufferedLogs.length > 0) {
    const errors = bufferedLogs.filter(log => log.level === 'ERROR').length;
    const timestamp = new Date().toLocaleTimeString();

    const summary = `[${timestamp}] Raccolti: ${bufferedLogs.length} elementi (Errori: ${errors})\n`;
    logDisplay.textContent = summary + logDisplay.textContent;

    console.log('Log raccolti:', bufferedLogs);
  }
});
```

## 📋 Utilizzo Type-Safe

Ecco un esempio di implementazione type-safe utilizzando i generics in TypeScript.

```ts
import { Observable, interval, timer } from 'rxjs';
import { bufferWhen, map } from 'rxjs';

interface MetricData {
  value: number;
  timestamp: Date;
  source: string;
}

interface BufferConfig {
  minDuration: number;
  maxDuration: number;
  adaptive: boolean;
}

class AdaptiveBuffer<T> {
  constructor(private config: BufferConfig) {}

  private getNextBufferDuration(previousCount: number): number {
    if (!this.config.adaptive) {
      return this.config.minDuration;
    }

    // Regola il periodo del buffer successivo in base al volume di dati
    const ratio = Math.min(previousCount / 10, 1);
    const duration =
      this.config.minDuration +
      (this.config.maxDuration - this.config.minDuration) * (1 - ratio);

    return Math.floor(duration);
  }

  apply(source$: Observable<T>): Observable<T[]> {
    let previousCount = 0;

    return source$.pipe(
      bufferWhen(() => {
        const duration = this.getNextBufferDuration(previousCount);
        return timer(duration);
      }),
      map(buffer => {
        previousCount = buffer.length;
        return buffer;
      })
    );
  }
}

// Esempio di utilizzo
const metricsStream$ = interval(300).pipe(
  map(i => ({
    value: Math.random() * 100,
    timestamp: new Date(),
    source: `sensor-${i % 3}`
  } as MetricData))
);

const buffer = new AdaptiveBuffer<MetricData>({
  minDuration: 1000,  // Minimo 1 secondo
  maxDuration: 5000,  // Massimo 5 secondi
  adaptive: true      // Adattivo
});

buffer.apply(metricsStream$).subscribe(metrics => {
  if (metrics.length > 0) {
    const avg = metrics.reduce((sum, m) => sum + m.value, 0) / metrics.length;
    console.log(`Dimensione buffer: ${metrics.length}, Media: ${avg.toFixed(2)}`);
  }
});
```

## 🎯 Confronto con Altri Operatori Buffer

```ts
import { interval, timer, Subject } from 'rxjs';
import { buffer, bufferTime, bufferCount, bufferWhen, bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0-9

// 1. buffer: Trigger fisso
const trigger$ = new Subject<void>();
source$.pipe(buffer(trigger$)).subscribe(console.log);
setInterval(() => trigger$.next(), 1000);
// Output: [0, 1, 2], [3, 4, 5], ... (al timing del trigger)

// 2. bufferTime: Intervallo di tempo fisso
source$.pipe(bufferTime(1000)).subscribe(console.log);
// Output: [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 3. bufferCount: Conteggio fisso
source$.pipe(bufferCount(3)).subscribe(console.log);
// Output: [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 4. bufferWhen: Controllo dinamico della fine (continuo)
source$.pipe(
  bufferWhen(() => timer(1000))
).subscribe(console.log);
// Output: [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 5. bufferToggle: Controllo indipendente inizio/fine (può sovrapporsi)
const opening$ = interval(1000);
const closing = () => timer(800);
source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Output: [3, 4, 5], [6, 7, 8]
```

| Operatore | Trigger | Controllo Dinamico | Overlap | Caso d'Uso |
|---|---|---|---|---|
| `buffer` | Observable esterno | ❌ | ❌ | Guidato da eventi |
| `bufferTime` | Tempo fisso | ❌ | ❌ | Aggregazione periodica |
| `bufferCount` | Conteggio fisso | ❌ | ❌ | Elaborazione quantitativa |
| `bufferWhen` | Dinamico (solo fine) | ✅ | ❌ | Elaborazione batch adattiva |
| `bufferToggle` | Dinamico (inizio e fine) | ✅ | ✅ | Gestione periodi complessi |

## ⚠️ Errori Comuni

> [!WARNING]
> La funzione di condizione di fine di `bufferWhen` **deve restituire un nuovo Observable ogni volta**. Se restituisce la stessa istanza di Observable, non funzionerà correttamente.

### Errore: Restituire la Stessa Istanza di Observable

```ts
import { interval, timer } from 'rxjs';
import { bufferWhen } from 'rxjs';

const source$ = interval(500);

// ❌ Esempio sbagliato: Riutilizzo della stessa istanza di Observable
const closingObservable = timer(1000);

source$.pipe(
  bufferWhen(() => closingObservable) // Non funzionerà dalla 2a volta in poi!
).subscribe(console.log);
// Solo il primo buffer viene emesso, nulla dopo
```

### Corretto: Restituisci un Nuovo Observable Ogni Volta

```ts
import { interval, timer } from 'rxjs';
import { bufferWhen } from 'rxjs';

const source$ = interval(500);

// ✅ Esempio corretto: Genera nuovo Observable ogni volta
source$.pipe(
  bufferWhen(() => timer(1000)) // Genera nuovo timer ogni volta
).subscribe(console.log);
// Output: [0, 1], [2, 3], [4, 5], ...
```

> [!IMPORTANT]
> La funzione `closingSelector` viene **sempre chiamata** ogni volta che il buffer precedente finisce, e ci si aspetta che restituisca un nuovo Observable.

## 🎓 Riepilogo

### Quando Usare bufferWhen
- ✅ Quando vuoi controllare dinamicamente la condizione di fine
- ✅ Quando sono necessari periodi di buffering continui
- ✅ Quando vuoi regolare il periodo successivo in base ai risultati del buffer precedente
- ✅ Quando vuoi implementare elaborazione batch adattiva

### Quando Usare bufferToggle
- ✅ Quando vuoi controllare inizio e fine indipendentemente
- ✅ Quando i periodi di buffer possono sovrapporsi
- ✅ Quando ci sono eventi chiari di inizio/fine, come pressioni di bottone

### Quando Usare bufferTime
- ✅ Quando è sufficiente il buffering a intervalli di tempo fissi
- ✅ Quando serve un'implementazione semplice

### Note
- ⚠️ `closingSelector` deve restituire un nuovo Observable ogni volta
- ⚠️ Condizioni di fine troppo complesse rendono difficile il debug
- ⚠️ Con controlli adattivi, il testing è importante per evitare comportamenti inaspettati

## 🚀 Prossimi Passi

- [buffer](/it/guide/operators/transformation/buffer) - Impara il buffering base
- [bufferTime](/it/guide/operators/transformation/bufferTime) - Impara il buffering basato sul tempo
- [bufferCount](/it/guide/operators/transformation/bufferCount) - Impara il buffering basato sul conteggio
- [bufferToggle](/it/guide/operators/transformation/bufferToggle) - Impara il buffering con controlli indipendenti di inizio e fine
- [Casi d'Uso Pratici Operatori di Trasformazione](/it/guide/operators/transformation/practical-use-cases) - Impara casi d'uso reali
