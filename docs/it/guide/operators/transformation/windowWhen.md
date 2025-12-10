---
description: L'operatore windowWhen è un operatore RxJS che controlla dinamicamente la condizione di fine e divide Observable. Abilita un'elaborazione stream continua dove la finestra successiva inizia immediatamente dopo la fine della finestra.
---

# windowWhen - Finestra con Controllo Dinamico della Fine

L'operatore `windowWhen` divide Observable con **controllo dinamico delle condizioni di fine**. Fornisce un pattern di elaborazione stream continua in cui la finestra successiva inizia immediatamente dopo la fine della finestra.

## 🔰 Sintassi e Utilizzo Base

```ts
import { interval } from 'rxjs';
import { windowWhen, mergeAll, take } from 'rxjs';

const source$ = interval(500); // Emetti valori ogni 0.5 secondi

// Condizione di fine: dopo 1 secondo
const closingSelector = () => interval(1000);

source$.pipe(
  windowWhen(closingSelector),
  take(4),
  mergeAll()
).subscribe(value => {
  console.log('Valore nella finestra:', value);
});

// Finestra 1: 0       (Inizia a 0 sec → Finisce a 1 sec)
// Finestra 2: 1, 2    (Inizia a 1 sec → Finisce a 2 sec)
// Finestra 3: 3, 4    (Inizia a 2 sec → Finisce a 3 sec)
// Finestra 4: 5, 6    (Inizia a 3 sec → Finisce a 4 sec)
```

**Flusso di operazione**:
1. La prima finestra inizia automaticamente
2. L'Observable restituito da `closingSelector()` emette un valore → La finestra finisce
3. **La finestra successiva inizia immediatamente**
4. Ripeti 2-3

[🌐 Documentazione Ufficiale RxJS - `windowWhen`](https://rxjs.dev/api/operators/windowWhen)

## 💡 Pattern di Utilizzo Tipici

- Raccolta dati a intervalli di tempo dinamici
- Elaborazione stream adattiva basata sul carico
- Controllo finestre basato sui risultati precedenti
- Raggruppamento dati continuo

## 🔍 Differenza da bufferWhen

| Operatore | Output | Caso d'Uso |
|:---|:---|:---|
| `bufferWhen` | **Array (T[])** | Elabora insieme i valori raggruppati |
| `windowWhen` | **Observable&lt;T&gt;** | Elaborazione stream diversa per ogni gruppo |

```ts
import { interval } from 'rxjs';
import { bufferWhen, windowWhen, mergeAll, take } from 'rxjs';

const source$ = interval(500);
const closing = () => interval(1000);

// bufferWhen - Output come array
source$.pipe(
  bufferWhen(closing),
  take(3)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Output: Buffer (array): [0]
  // Output: Buffer (array): [1, 2]
  // Output: Buffer (array): [3, 4]
});

// windowWhen - Output come Observable
source$.pipe(
  windowWhen(closing),
  take(3),
  mergeAll()
).subscribe(value => {
  console.log('Valore nella finestra:', value);
  // Output: Valore nella finestra: 0
  // Output: Valore nella finestra: 1
  // Output: Valore nella finestra: 2
  // ...
});
```

## 🧠 Esempio di Codice Pratico 1: Raccolta Dati a Intervalli di Tempo Dinamici

Questo è un esempio di regolazione del periodo della finestra successiva in base ai risultati della finestra precedente.

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, mergeMap, toArray, scan, map } from 'rxjs';

// Dati sensore (sempre in generazione)
const sensorData$ = interval(100).pipe(
  map(() => ({
    timestamp: Date.now(),
    temperature: 20 + Math.random() * 10 // 20-30 gradi
  }))
);

let windowNumber = 0;
let previousAvgTemp = 25;

sensorData$.pipe(
  windowWhen(() => {
    const current = ++windowNumber;
    // Intervallo più corto quando la temperatura è più alta
    const duration = previousAvgTemp > 27 ? 500 : 1000;
    console.log(`Finestra ${current} iniziata (durata: ${duration}ms)`);
    return timer(duration);
  }),
  mergeMap(window$ => {
    const currentWindow = windowNumber;  // Mantieni numero finestra corrente
    return window$.pipe(
      toArray(),
      map(data => {
        const avgTemp = data.reduce((sum, d) => sum + d.temperature, 0) / data.length;
        previousAvgTemp = avgTemp;
        return {
          window: currentWindow,  // Usa numero finestra mantenuto
          count: data.length,
          avgTemp
        };
      })
    );
  })
).subscribe(stats => {
  console.log(`Finestra ${stats.window}: Temp media ${stats.avgTemp.toFixed(1)}°C, ${stats.count} campioni`);
});
```

## 🎯 Esempio di Codice Pratico 2: Elaborazione Stream Adattiva Basata sul Carico

Questo è un esempio di cambiamento dinamico della lunghezza della finestra in base al carico del sistema.

```ts
import { interval, timer, fromEvent } from 'rxjs';
import { windowWhen, mergeMap, scan, map } from 'rxjs';

// Crea area di output
const container = document.createElement('div');
document.body.appendChild(container);

const loadButton = document.createElement('button');
loadButton.textContent = 'Genera Carico';
container.appendChild(loadButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = 'Carico basso: Raccogli a intervalli di 5 secondi';
container.appendChild(status);

const logDisplay = document.createElement('div');
logDisplay.style.marginTop = '10px';
logDisplay.style.maxHeight = '300px';
logDisplay.style.overflow = 'auto';
container.appendChild(logDisplay);

// Stream log (sempre in generazione)
let logCounter = 0;
const logs$ = interval(200).pipe(
  map(() => ({
    id: logCounter++,
    level: Math.random() > 0.7 ? 'ERROR' : 'INFO',
    timestamp: new Date()
  }))
);

// Livello di carico
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
  const interval = getWindowDuration(loadLevel);
  const loadText = loadLevel === 0 ? 'Carico basso' :
                   loadLevel <= 2 ? 'Carico medio' : 'Carico alto';
  status.textContent = `${loadText} (Livello ${loadLevel}): Raccogli a intervalli di ${interval / 1000} secondi`;
}

function getWindowDuration(load: number): number {
  // Carico più alto = intervallo più corto
  switch (load) {
    case 0: return 5000;
    case 1: return 3000;
    case 2: return 2000;
    case 3: return 1000;
    case 4: return 500;
    default: return 300;
  }
}

let windowNum = 0;

// Elaborazione finestre adattiva
logs$.pipe(
  windowWhen(() => {
    windowNum++;
    return timer(getWindowDuration(loadLevel));
  }),
  mergeMap(window$ =>
    window$.pipe(
      scan((stats, log) => ({
        count: stats.count + 1,
        errors: stats.errors + (log.level === 'ERROR' ? 1 : 0),
        window: windowNum
      }), { count: 0, errors: 0, window: windowNum })
    )
  )
).subscribe(stats => {
  const timestamp = new Date().toLocaleTimeString();
  const div = document.createElement('div');
  div.textContent = `[${timestamp}] Finestra ${stats.window}: ${stats.count} elementi (Errori: ${stats.errors})`;
  logDisplay.insertBefore(div, logDisplay.firstChild);
});
```

## 🆚 Differenza da windowToggle

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, windowToggle, mergeAll } from 'rxjs';

const source$ = interval(200);

// windowWhen: Controlla solo la fine (il prossimo inizia subito dopo la fine)
source$.pipe(
  windowWhen(() => timer(1000)),
  mergeAll()
).subscribe();

// windowToggle: Controllo separato di inizio e fine
source$.pipe(
  windowToggle(
    interval(1000),          // Trigger di inizio
    () => timer(500)         // Trigger di fine (500ms dopo l'inizio)
  ),
  mergeAll()
).subscribe();
```

| Operatore | Controllo | Periodo Finestra | Caso d'Uso |
|:---|:---|:---|:---|
| `windowWhen(closing)` | Solo controllo fine | Continuo | Finestra periodica semplice |
| `windowToggle(open$, close)` | Controllo separato inizio/fine | Può sovrapporsi | Condizioni complesse inizio/fine |

**Linee guida di utilizzo**:
- **`windowWhen`**: Elabora tutti i dati continuamente senza omissioni (logging, aggregazione dati, ecc.)
- **`windowToggle`**: Elabora dati solo per un periodo specifico (durante orario lavorativo, pressioni bottone, ecc.)

## 🎯 Esempio Pratico: Controllo Dimensione Finestra Adattiva

Ecco un esempio di regolazione automatica del periodo della finestra successiva in base ai risultati della finestra precedente.

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, mergeMap, toArray, map } from 'rxjs';

interface WindowStats {
  count: number;
  nextDuration: number;
}

const data$ = interval(100);

let previousCount = 0;

// Regola il periodo della finestra successiva in base al volume di dati
function getNextDuration(count: number): number {
  if (count > 20) {
    return 500;  // Volume dati alto → Intervallo corto
  } else if (count > 10) {
    return 1000; // Medio → Intervallo medio
  } else {
    return 2000; // Volume dati basso → Intervallo lungo
  }
}

data$.pipe(
  windowWhen(() => timer(getNextDuration(previousCount))),
  mergeMap(window$ =>
    window$.pipe(
      toArray(),
      map(data => {
        previousCount = data.length;
        return {
          count: data.length,
          nextDuration: getNextDuration(data.length)
        } as WindowStats;
      })
    )
  )
).subscribe(stats => {
  console.log(`Dimensione finestra: ${stats.count} elementi, Prossima durata: ${stats.nextDuration}ms`);
});
```

## ⚠️ Note

### 1. Gestione Subscription delle Finestre

Ogni finestra è un Observable indipendente, quindi devi sottoscriverlo esplicitamente o appiattirlo con `mergeAll()` o simili.

```ts
source$.pipe(
  windowWhen(closing)
).subscribe(window$ => {
  // I valori non fluiranno a meno che non ti iscrivi alla finestra stessa
  window$.subscribe(value => {
    console.log('Valore:', value);
  });
});
```

### 2. Necessario Restituire un Nuovo Observable Ogni Volta

La funzione `closingSelector` **deve restituire un nuovo Observable ogni volta**. Se restituisce la stessa istanza, non funzionerà correttamente.

```ts
// ❌ Esempio sbagliato: Riutilizzo della stessa istanza di Observable
const closingObservable = timer(1000);

source$.pipe(
  windowWhen(() => closingObservable) // Non funzionerà dalla 2a volta in poi!
).subscribe();

// ✅ Esempio corretto: Genera nuovo Observable ogni volta
source$.pipe(
  windowWhen(() => timer(1000)) // Genera nuovo timer ogni volta
).subscribe();
```

### 3. Attenzione alle Condizioni di Fine Troppo Complesse

Condizioni di fine troppo complesse possono rendere difficile il debug.

```ts
// Esempio troppo complesso
let counter = 0;
source$.pipe(
  windowWhen(() => {
    counter++;
    const duration = counter % 3 === 0 ? 500 :
                     counter % 2 === 0 ? 1000 : 1500;
    return timer(duration);
  })
).subscribe();
// Difficile da debuggare
```

## 🆚 Confronto degli Operatori window

| Operatore | Controllo | Periodo Finestra | Caso d'Uso |
|:---|:---|:---|:---|
| `window` | Un altro Observable emette | Continuo | Partizionamento guidato da eventi |
| `windowTime` | Intervallo di tempo fisso | Continuo | Partizionamento basato sul tempo |
| `windowCount` | Conteggio fisso | Continuo | Partizionamento basato sul conteggio |
| `windowToggle` | Controllo separato inizio/fine | Può sovrapporsi | Condizioni complesse inizio/fine |
| `windowWhen` | **Solo controllo dinamico fine** | **Continuo** | **Elaborazione finestre adattiva** |

## 📚 Operatori Correlati

- [bufferWhen](/it/guide/operators/transformation/bufferWhen) - Raccogli valori come array (versione array di windowWhen)
- [window](/it/guide/operators/transformation/window) - Dividi finestra a timing di Observable diversi
- [windowTime](/it/guide/operators/transformation/windowTime) - Divisione finestre basata sul tempo
- [windowCount](/it/guide/operators/transformation/windowCount) - Divisione finestre basata sul conteggio
- [windowToggle](/it/guide/operators/transformation/windowToggle) - Controllo finestre con Observable di inizio e fine

## Riepilogo

L'operatore `windowWhen` è uno strumento utile per il controllo dinamico delle condizioni di fine e l'elaborazione continua delle finestre.

- ✅ Le condizioni di fine possono essere controllate dinamicamente
- ✅ Elaborazione finestre continua (senza perdita dati)
- ✅ Può regolare la finestra successiva in base ai risultati precedenti
- ⚠️ Richiede gestione subscription
- ⚠️ Necessario restituire un nuovo Observable ogni volta
- ⚠️ Fai attenzione a non complicare troppo le condizioni di fine
