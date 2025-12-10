---
description: windowCount è un operatore di conversione RxJS che divide un Observable per un numero specificato di elementi. È ideale per elaborazione stream basata sul conteggio, aggregazione per conteggio fisso ed elaborazione paginazione. A differenza di bufferCount, può applicare elaborazione indipendente a ogni finestra. L'inferenza dei tipi di TypeScript permette la divisione finestre e le operazioni stream type-safe.
titleTemplate: ':title | RxJS'
---

# windowCount - Dividi Observable per Conteggio Specificato

L'operatore `windowCount` **divide** i valori emessi in nuovi Observable per ogni conteggio specificato.
Mentre `bufferCount` restituisce un array, `windowCount` restituisce un **Observable&lt;T&gt;**, permettendo di applicare operatori aggiuntivi a ogni finestra.

## 🔰 Sintassi e Utilizzo Base

```ts
import { interval } from 'rxjs';
import { windowCount, mergeAll } from 'rxjs';

// Emetti valori ogni 100ms
const source$ = interval(100);

source$.pipe(
  windowCount(5),
  mergeAll() // Appiattisci ogni finestra
).subscribe(value => {
  console.log('Valore nella finestra:', value);
});

// Output:
// Valore nella finestra: 0
// Valore nella finestra: 1
// Valore nella finestra: 2
// Valore nella finestra: 3
// Valore nella finestra: 4
// (Nuova finestra inizia)
// Valore nella finestra: 5
// ...
```

- Una nuova finestra (Observable) viene creata ogni 5 valori.
- È unico in quanto divide su base di conteggio.

[🌐 Documentazione Ufficiale RxJS - `windowCount`](https://rxjs.dev/api/operators/windowCount)

## 💡 Pattern di Utilizzo Tipici

- Elaborazione aggregata per ogni conteggio fisso
- Trasmissione batch di dati (elaborazione diversa per ogni finestra)
- Elaborazione paginazione
- Calcola statistiche per finestra

## 🔍 Differenza da bufferCount

| Operatore | Output | Caso d'Uso |
|:---|:---|:---|
| `bufferCount` | **Array (T[])** | Elabora insieme i valori raggruppati |
| `windowCount` | **Observable&lt;T&gt;** | Elaborazione stream diversa per ogni gruppo |

```ts
import { interval } from 'rxjs';
import { bufferCount, windowCount, mergeAll } from 'rxjs';

const source$ = interval(100);

// bufferCount - Output come array
source$.pipe(
  bufferCount(5)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Output: Buffer (array): [0, 1, 2, 3, 4]
});

// windowCount - Output come Observable
source$.pipe(
  windowCount(5)
).subscribe(window$ => {
  console.log('Window (Observable):', window$);
  window$.subscribe(value => {
    console.log('  Valore nella finestra:', value);
  });
});
```

## 🧠 Esempio di Codice Pratico 1: Somma per Finestra

Questo è un esempio di calcolo della somma di ogni 5 valori.

```ts
import { interval } from 'rxjs';
import { windowCount, map, mergeAll, reduce } from 'rxjs';

// Crea area di output
const output = document.createElement('div');
output.innerHTML = '<h3>Somma Ogni 5 Valori</h3>';
document.body.appendChild(output);

const source$ = interval(200);

let windowNumber = 0;

source$.pipe(
  windowCount(5),
  map(window$ => {
    const current = ++windowNumber;
    console.log(`Finestra ${current} iniziata`);

    // Calcola somma per ogni finestra
    return window$.pipe(
      reduce((sum, value) => sum + value, 0),
      map(sum => ({ windowNum: current, sum }))  // Includi numero finestra
    );
  }),
  mergeAll()
).subscribe(result => {
  const div = document.createElement('div');
  div.textContent = `Somma finestra ${result.windowNum}: ${result.sum}`;
  output.appendChild(div);
});

// Output:
// Somma finestra 1: 10  (0+1+2+3+4)
// Somma finestra 2: 35  (5+6+7+8+9)
// Somma finestra 3: 60  (10+11+12+13+14)
```

## 🎯 Esempio di Codice Pratico 2: Specificare Indice di Inizio

Puoi specificare un indice di partenza con il secondo argomento. Questo crea finestre sovrapposte.

```ts
import { range } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

// Emetti valori da 0 a 9
range(0, 10).pipe(
  windowCount(3, 2), // 3 elementi ciascuno, inizio spostato di 2
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Finestra:', values);
});

// Output:
// Finestra: [0, 1, 2]
// Finestra: [2, 3, 4]    ← Iniziato spostato di 2 (da 2)
// Finestra: [4, 5, 6]    ← Iniziato spostato di 2 (da 4)
// Finestra: [6, 7, 8]
// Finestra: [8, 9]       ← Ultimi 2 elementi
```

### Pattern di Operazione Indice di Inizio

```ts
// windowCount(bufferSize, startBufferEvery)

windowCount(3, 3) // Continuo (default): [0,1,2], [3,4,5], [6,7,8]
windowCount(3, 2) // Overlap: [0,1,2], [2,3,4], [4,5,6]
windowCount(3, 4) // Con gap: [0,1,2], [4,5,6], [8,9,10]
```

## 🎯 Esempio Pratico: Elaborazione Diversa per Ogni Finestra

```ts
import { interval } from 'rxjs';
import { windowCount, map, mergeAll, take } from 'rxjs';

const source$ = interval(100);
let windowNumber = 0;

source$.pipe(
  windowCount(5),
  map(window$ => {
    const current = ++windowNumber;

    if (current % 2 === 0) {
      // Finestre pari: Ottieni solo i primi 2 elementi
      console.log(`Finestra ${current}: Ottieni primi 2 elementi`);
      return window$.pipe(take(2));
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

## 🧠 Esempio di Codice Pratico 3: Elaborazione Tipo Paginazione

```ts
import { from } from 'rxjs';
import { windowCount, mergeMap, toArray, map } from 'rxjs';

// Dati da 1-20
const data$ = from(Array.from({ length: 20 }, (_, i) => i + 1));

// Paginazione di 5 elementi
data$.pipe(
  windowCount(5),
  mergeMap((window$, index) => {
    const pageNumber = index + 1;
    return window$.pipe(
      toArray(),
      map(items => ({ page: pageNumber, items }))
    );
  })
).subscribe(page => {
  console.log(`Pagina ${page.page}:`, page.items);
});

// Output:
// Pagina 1: [1, 2, 3, 4, 5]
// Pagina 2: [6, 7, 8, 9, 10]
// Pagina 3: [11, 12, 13, 14, 15]
// Pagina 4: [16, 17, 18, 19, 20]
```

## ⚠️ Note

### 1. Gestione Subscription delle Finestre

Ogni finestra è un Observable indipendente e deve essere esplicitamente sottoscritta.

```ts
source$.pipe(
  windowCount(5)
).subscribe(window$ => {
  // I valori non fluiranno a meno che non ti iscrivi alla finestra stessa
  window$.subscribe(value => {
    console.log('Valore:', value);
  });
});
```

Oppure usa `mergeAll()`, `concatAll()`, `switchAll()`, ecc. per appiattire.

### 2. Ultima Finestra

Al completamento dell'Observable sorgente, l'ultima finestra viene emessa anche se contiene meno del numero specificato di elementi.

```ts
import { of } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

of(1, 2, 3, 4, 5, 6, 7).pipe(
  windowCount(3),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Finestra:', values);
});

// Output:
// Finestra: [1, 2, 3]
// Finestra: [4, 5, 6]
// Finestra: [7]  ← Solo 1 elemento
```

### 3. Utilizzo Memoria per Indice di Inizio

Se `startBufferEvery` è minore di `bufferSize` (overlap), più finestre saranno attive contemporaneamente, aumentando l'utilizzo di memoria.

```ts
// Overlap: Massimo 2 finestre attive simultaneamente
windowCount(5, 3)

// Contromisura: Limita con take() se necessario
source$.pipe(
  take(100), // Massimo 100 elementi
  windowCount(5, 3)
)
```

## 🆚 Confronto degli Operatori window

| Operatore | Timing del Delimitatore | Caso d'Uso |
|:---|:---|:---|
| `window` | Un altro Observable emette | Partizionamento guidato da eventi |
| `windowTime` | Intervallo di tempo fisso | Partizionamento basato sul tempo |
| `windowCount` | **Conteggio fisso** | **Partizionamento basato sul conteggio** |
| `windowToggle` | Observable di inizio e fine | Controllo dinamico inizio/fine |
| `windowWhen` | Condizione di chiusura dinamica | Condizione di fine diversa per finestra |

## 📚 Operatori Correlati

- [`bufferCount`](/it/guide/operators/transformation/bufferCount) - Raccogli valori come array (versione array di windowCount)
- [`window`](/it/guide/operators/transformation/window) - Dividi finestra a timing di Observable diversi
- [`windowTime`](/it/guide/operators/transformation/windowTime) - Divisione finestre basata sul tempo
- [`windowToggle`](/it/guide/operators/transformation/windowToggle) - Controllo finestre con Observable di inizio e fine
- [`windowWhen`](/it/guide/operators/transformation/windowWhen) - Divisione finestre con condizioni di chiusura dinamiche

## Riepilogo

L'operatore `windowCount` è uno strumento utile per partizionare stream su base di conteggio e trattare ogni gruppo come un Observable indipendente.

- ✅ Ideale per aggregazione ed elaborazione per conteggio fisso
- ✅ Elaborazione diversa può essere applicata a ogni finestra
- ✅ Può essere sovrapposto per indice di inizio
- ⚠️ Richiede gestione subscription
- ⚠️ Fai attenzione all'utilizzo memoria quando sovrapponi
