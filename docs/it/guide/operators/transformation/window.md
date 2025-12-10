---
description: window è un operatore RxJS che divide un Observable sorgente in Observable annidati quando un altro Observable emette valori, ideale per elaborazione avanzata di stream guidata da eventi.
titleTemplate: ':title | RxJS'
---

# window - Dividi Observable al Timing di un Altro Observable

L'operatore `window` raggruppa i valori di un Observable sorgente **fino a quando un altro Observable emette valori** ed emette quel gruppo come un **nuovo Observable**.
Mentre `buffer` restituisce un array, `window` restituisce un **Observable&lt;T&gt;**, permettendo di applicare ulteriori operatori a ogni finestra.

## 🔰 Sintassi e Utilizzo Base

```ts
import { interval, fromEvent } from 'rxjs';
import { window, mergeAll } from 'rxjs';

// Emetti valori ogni 100ms
const source$ = interval(100);

// Usa l'evento click come trigger
const clicks$ = fromEvent(document, 'click');

source$.pipe(
  window(clicks$),
  mergeAll() // Appiattisci ogni finestra
).subscribe(value => {
  console.log('Valore nella finestra:', value);
});

// Una nuova finestra inizia con ogni click
```

- Ogni volta che `clicks$` emette un valore, viene creata una nuova finestra (Observable).
- Ogni finestra può essere trattata come un Observable indipendente.

[🌐 Documentazione Ufficiale RxJS - `window`](https://rxjs.dev/api/operators/window)

## 💡 Pattern di Utilizzo Tipici

- Partizionamento stream guidato da eventi
- Applicare elaborazione diversa a ogni finestra
- Raggruppamento dati con delimitazione dinamica
- Elaborazione aggregata per ogni finestra

## 🔍 Differenza da buffer

| Operatore | Output | Caso d'Uso |
|:---|:---|:---|
| `buffer` | **Array (T[])** | Elabora insieme i valori raggruppati |
| `window` | **Observable&lt;T&gt;** | Elaborazione stream diversa per ogni gruppo |

```ts
import { interval, timer } from 'rxjs';
import { buffer, window, mergeAll } from 'rxjs';

const source$ = interval(100);
const trigger$ = timer(1000, 1000);

// buffer - Output come array
source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Output: Buffer (array): [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
});

// window - Output come Observable
source$.pipe(
  window(trigger$)
).subscribe(window$ => {
  console.log('Window (Observable):', window$);
  window$.subscribe(value => {
    console.log('  Valore nella finestra:', value);
  });
});
```

## 🧠 Esempio di Codice Pratico 1: Conteggio per Finestra

Questo esempio attiva il click del bottone e conta il numero di eventi fino a quel punto.

```ts
import { interval, fromEvent } from 'rxjs';
import { window, map, mergeAll, scan } from 'rxjs';

// Crea bottone
const button = document.createElement('button');
button.textContent = 'Delimita Finestra';
document.body.appendChild(button);

// Area di output
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Emetti valori ogni 100ms
const source$ = interval(100);

// Trigger al click del bottone
const clicks$ = fromEvent(button, 'click');

let windowCount = 0;

source$.pipe(
  window(clicks$),
  map(window$ => {
    const currentWindow = ++windowCount;
    console.log(`Finestra ${currentWindow} iniziata`);

    // Conta i valori in ogni finestra
    return window$.pipe(
      scan((count) => count + 1, 0)
    );
  }),
  mergeAll()
).subscribe(count => {
  output.textContent = `Finestra corrente: ${windowCount}, Conteggio: ${count}`;
});
```

- Ogni volta che si clicca un bottone, viene creata una nuova finestra.
- Il numero di valori in ogni finestra viene contato in tempo reale.

## 🎯 Esempio di Codice Pratico 2: Elaborazione Diversa per Ogni Finestra

Questo è un esempio avanzato che applica elaborazione diversa a ogni finestra.

```ts
import { interval, fromEvent } from 'rxjs';
import { window, take, mergeAll, map } from 'rxjs';

const source$ = interval(200);
const clicks$ = fromEvent(document, 'click');

let windowNumber = 0;

source$.pipe(
  window(clicks$),
  map(window$ => {
    const current = ++windowNumber;

    if (current % 2 === 0) {
      // Finestre pari: Ottieni solo i primi 3 elementi
      console.log(`Finestra ${current}: Ottieni primi 3 elementi`);
      return window$.pipe(take(3));
    } else {
      // Finestre dispari: Ottieni tutti
      console.log(`Finestra ${current}: Ottieni tutti`);
      return window$;
    }
  }),
  mergeAll()
).subscribe(value => {
  console.log(`Valore: ${value} (Finestra ${windowNumber})`);
});
```

- Puoi applicare condizionalmente elaborazione diversa per ogni finestra.
- Ogni finestra è un Observable indipendente, quindi puoi combinare liberamente gli operatori.

## 🎯 Esempio Pratico: Controllo con Trigger Multipli

```ts
import { interval, merge, fromEvent, timer } from 'rxjs';
import { window, mergeAll, scan, map } from 'rxjs';

const source$ = interval(100);

// Trigger multipli: click o 3 secondi trascorsi
const clicks$ = fromEvent(document, 'click');
const threeSeconds$ = timer(3000, 3000);
const trigger$ = merge(clicks$, threeSeconds$);

source$.pipe(
  window(trigger$),
  map((window$, index) => {
    console.log(`Finestra ${index + 1} iniziata`);

    // Calcola la somma per ogni finestra
    return window$.pipe(
      scan((sum, value) => sum + value, 0)
    );
  }),
  mergeAll()
).subscribe(sum => {
  console.log('Somma corrente:', sum);
});
```

## ⚠️ Note

### 1. Gestione Subscription delle Finestre

Ogni finestra è un Observable indipendente, quindi deve essere sottoscritto esplicitamente.

```ts
source$.pipe(
  window(trigger$)
).subscribe(window$ => {
  // I valori non fluiranno a meno che non ti iscrivi alla finestra stessa
  window$.subscribe(value => {
    console.log('Valore:', value);
  });
});
```

Oppure usa `mergeAll()`, `concatAll()`, `switchAll()`, ecc. per appiattire.

```ts
source$.pipe(
  window(trigger$),
  mergeAll() // Unisci tutte le finestre
).subscribe(value => {
  console.log('Valore:', value);
});
```

### 2. Attenzione ai Memory Leak

**Problema**: Se l'Observable trigger non emette valori, la prima finestra rimane aperta per sempre e i valori si accumulano all'infinito.

#### ❌ Esempio Sbagliato: Il Trigger Non Si Verifica

```ts
import { interval, fromEvent } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100); // Continua a emettere valori ogni 100ms

// Il bottone non esiste, o l'utente non clicca
const button = document.querySelector('#start-button'); // Possibilmente null
const clicks$ = fromEvent(button, 'click'); // Errore o non si attiva mai

source$.pipe(
  window(clicks$), // La prima finestra non si chiuderà se clicks$ non si attiva
  mergeAll()
).subscribe();

// Problemi:
// - Se clicks$ non emette, la prima finestra rimane aperta
// - I valori di source$ (0, 1, 2, 3...) continuano ad accumularsi in memoria
// - Causa memory leak
```

#### ✅ Esempio Corretto 1: Imposta Timeout

Imposta un timeout per evitare che la prima finestra rimanga aperta troppo a lungo.

```ts
import { interval, fromEvent, timer, merge } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100);
const button = document.querySelector('#start-button');
const clicks$ = button ? fromEvent(button, 'click') : interval(0); // fallback

// Chiudi finestra al click o dopo 5 secondi, qualunque venga prima
const autoClose$ = timer(5000); // Emetti dopo 5 secondi
const trigger$ = merge(clicks$, autoClose$);

source$.pipe(
  window(trigger$), // La finestra si chiuderà sempre entro 5 secondi
  mergeAll()
).subscribe();
```

#### ✅ Esempio Corretto 2: Chiudi Finestre Periodicamente

Chiudi le finestre periodicamente anche senza click.

```ts
import { interval, fromEvent, timer, merge } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100);
const button = document.querySelector('#start-button');
const clicks$ = fromEvent(button, 'click');

// Chiudi finestra al click o ogni 3 secondi
const autoClose$ = timer(3000, 3000); // Dopo i primi 3 secondi, poi ogni 3 secondi
const trigger$ = merge(clicks$, autoClose$);

source$.pipe(
  window(trigger$), // La finestra si chiude ogni 3 secondi anche senza click
  mergeAll()
).subscribe();

// Risultato:
// - Le finestre si chiudono automaticamente ogni 3 secondi anche senza click dell'utente
// - Previene l'accumulo infinito di valori in memoria
```

### 3. Overlap delle Finestre

Per default, le finestre non si sovrappongono (la finestra successiva inizia dopo la chiusura della precedente).
Se serve overlap, usa `windowToggle` o `windowWhen`.

## 🆚 Confronto degli Operatori window

| Operatore | Timing del Delimitatore | Caso d'Uso |
|:---|:---|:---|
| `window` | Un altro Observable emette | Partizionamento guidato da eventi |
| `windowTime` | Intervallo di tempo fisso | Partizionamento basato sul tempo |
| `windowCount` | Conteggio fisso | Partizionamento basato sul conteggio |
| `windowToggle` | Observable di inizio e fine | Controllo dinamico inizio/fine |
| `windowWhen` | Condizione di chiusura dinamica | Condizione di fine diversa per finestra |

## 📚 Operatori Correlati

- [`buffer`](/it/guide/operators/transformation/buffer) - Raccogli valori come array (versione array di window)
- [`windowTime`](/it/guide/operators/transformation/windowTime) - Partizionamento finestre basato sul tempo
- [`windowCount`](/it/guide/operators/transformation/windowCount) - Partizionamento finestre basato sul conteggio
- [`windowToggle`](/it/guide/operators/transformation/windowToggle) - Controllo finestre con Observable di inizio e fine
- [`windowWhen`](/it/guide/operators/transformation/windowWhen) - Partizionamento finestre con condizione di chiusura dinamica
- [`groupBy`](/it/guide/operators/transformation/groupBy) - Raggruppa Observable per chiave

## Riepilogo

L'operatore `window` è uno strumento potente che divide gli stream attivati da un Observable esterno e può elaborare ogni gruppo come un Observable indipendente.

- ✅ Può applicare elaborazione diversa a ogni finestra
- ✅ Controllo flessibile guidato da eventi
- ✅ Supporta operazioni stream avanzate
- ⚠️ Richiede gestione subscription
- ⚠️ Attenzione ai memory leak
