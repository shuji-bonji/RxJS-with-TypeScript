---
description: "range() - Funzione di Creazione che genera numeri interi consecutivi dichiarativamente: Alternativa efficiente in memoria ai cicli for per elaborazione batch e paginazione"
---

# range() - Genera un range di numeri

`range()` è una Funzione di Creazione simile a for che emette un numero specificato di interi consecutivi da un valore iniziale specificato.

## Panoramica

`range()` emette una sequenza di interi consecutivi come Observable specificando un valore iniziale e il numero di interi. È usata per generazione numeri sequenziali ed elaborazione batch come modo dichiarativo per sostituire l'istruzione `for` tradizionale.

**Firma**:
```typescript
function range(
  start: number,
  count?: number,
  scheduler?: SchedulerLike
): Observable<number>
```

**Parametri**:
- `start`: Il valore iniziale (da cui iniziare a emettere)
- `count`: il numero di valori da pubblicare (se omesso, da 0 a meno di `start`)
- `scheduler`: lo scheduler per emettere i valori (se omesso: emette sincronamente)

**Documentazione Ufficiale**: [📘 RxJS Ufficiale: range()](https://rxjs.dev/api/index/function/range)

## Uso Base

### Pattern 1: Specifica valore iniziale e conteggio

Questo è l'utilizzo più comune.

```typescript
import { range } from 'rxjs';

// Genera 5 numeri sequenziali da 1 (1, 2, 3, 4, 5)
range(1, 5).subscribe({
  next: value => console.log('Valore:', value),
  complete: () => console.log('Completo')
});

// Output:
// Valore: 1
// Valore: 2
// Valore: 3
// Valore: 4
// Valore: 5
// Completo
```

### Pattern 2: Numeri sequenziali da 0

Impostando il valore iniziale a 0, può essere generato un numero sequenziale come indice array.

```typescript
import { range } from 'rxjs';

// Da 0 a 10 numeri sequenziali (0, 1, 2, ..., 9)
range(0, 10).subscribe(console.log);
// Output: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
```

### Pattern 3: Inizia con numero negativo

Possono essere generati anche numeri negativi.

```typescript
import { range } from 'rxjs';

// 5 numeri sequenziali da -3 (-3, -2, -1, 0, 1)
range(-3, 5).subscribe(console.log);
// Output: -3, -2, -1, 0, 1
```

## Caratteristiche Importanti

### 1. Emissione Sincrona

Per default, `range()` emette tutti i valori **sincronamente** alla subscription.

```typescript
import { range } from 'rxjs';

console.log('Prima della subscription');

range(1, 3).subscribe(value => console.log('Valore:', value));

console.log('Dopo la subscription');

// Output:
// Prima della subscription
// Valore: 1
// Valore: 2
// Valore: 3
// Dopo la subscription
```

### 2. Completa Immediatamente

Notifica `complete` immediatamente dopo aver pubblicato tutti i valori.

```typescript
import { range } from 'rxjs';

range(1, 3).subscribe({
  next: val => console.log(val),
  complete: () => console.log('Completo!')
});

// Output: 1, 2, 3, Completo!
```

### 3. Equivalenza con istruzione for

`range(start, count)` è equivalente alla seguente istruzione for.

```typescript
// Istruzione for imperativa
for (let i = start; i < start + count; i++) {
  console.log(i);
}

// range() dichiarativo
range(start, count).subscribe(console.log);
```

## Casi d'Uso Pratici

### 1. Elaborazione Batch

Usato per eseguire task multipli sequenzialmente.

```typescript
import { range, of, Observable, concatMap, delay, map } from 'rxjs';
// Funzione per simulare elaborazione dati
function processItem(index: number): Observable<string> {
  return of(index).pipe(
    delay(100), // Simula 100ms tempo elaborazione
    map(i => `Risultato elaborazione elemento ${i}`)
  );
}

// Elabora sequenzialmente 10 elementi dati (1 secondo delay tra ogni processo)
range(1, 10).pipe(
  concatMap(index =>
    processItem(index).pipe(delay(1000))
  )
).subscribe({
  next: result => console.log(`Elaborazione completa: ${result}`),
  complete: () => console.log('Tutta l\'elaborazione completata')
});

// Output:
// Elaborazione completa: Risultato elaborazione elemento 1 (dopo circa 1.1 secondi)
// Elaborazione completa: Risultato elaborazione elemento 2 (dopo circa 2.1 secondi)
// ...
// Elaborazione completa: Risultato elaborazione elemento 10 (dopo circa 10.1 sec.)
// Tutta l'elaborazione completata
```

### 2. Paginazione

Recupera pagine multiple di dati sequenzialmente.

```typescript
import { range, of, Observable, concatMap, delay } from 'rxjs';
interface PageData {
  page: number;
  items: string[];
}

// Funzione per simulare recupero dati pagina
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`Elemento${page}-1`, `Elemento${page}-2`, `Elemento${page}-3`]
  }).pipe(
    delay(500) // Simula chiamata API
  );
}

function fetchAllPages(totalPages: number) {
  return range(1, totalPages).pipe(
    concatMap(page => fetchPage(page))
  );
}

fetchAllPages(5).subscribe({
  next: (data: PageData) => console.log(`Pagina ${data.page}:`, data.items),
  complete: () => console.log('Tutte le pagine recuperate')
});

// Output:
// Pagina 1: ['Elemento1-1', 'Elemento1-2', 'Elemento1-3']
// Pagina 2: ['Elemento2-1', 'Elemento2-2', 'Elemento2-3']
// Pagina 3: ['Elemento3-1', 'Elemento3-2', 'Elemento3-3']
// Pagina 4: ['Elemento4-1', 'Elemento4-2', 'Elemento4-3']
// Pagina 5: ['Elemento5-1', 'Elemento5-2', 'Elemento5-3']
// Tutte le pagine recuperate
```

### 3. Elaborazione Indici Array

Usa come loop basato su indici quando elabori ogni elemento di un array.

```typescript
import { range, map } from 'rxjs';
const items = ['Mela', 'Banana', 'Ciliegia', 'Dattero', 'Sambuco'];

range(0, items.length).pipe(
  map(index => ({ index, item: items[index] }))
).subscribe(({ index, item }) => {
  console.log(`[${index}] ${item}`);
});

// Output:
// [0] Mela
// [1] Banana
// [2] Ciliegia
// [3] Dattero
// [4] Sambuco
```

### 4. Generazione Dati Test

Utile per generare dati mock per unit test.

```typescript
import { range, map, toArray } from 'rxjs';
// Genera dati utente mock
range(1, 100).pipe(
  map(id => ({
    id,
    name: `Utente${id}`,
    email: `utente${id}@example.com`
  })),
  toArray()
).subscribe(users => {
  console.log(`${users.length} utenti generati`);
  // Usa nei test
});
```

## Asincronizzazione con Scheduler

Quando elabori grandi quantità di dati, l'esecuzione asincrona è possibile specificando uno scheduler.

```typescript
import { range, asyncScheduler, observeOn } from 'rxjs';
console.log('Inizio');

// Emetti 1.000.000 numeri asincronamente
range(1, 1000000).pipe(
  observeOn(asyncScheduler)
).subscribe({
  next: val => {
    if (val % 100000 === 0) {
      console.log(`Progresso: ${val}`);
    }
  },
  complete: () => console.log('Completo')
});

console.log('Dopo subscription (asincrono, quindi eseguito immediatamente)');

// Output:
// Inizio
// Dopo subscription (asincrono, quindi eseguito immediatamente)
// Progresso: 100000
// Progresso: 200000
// ...
// Completo
```

> [!TIP]
> **Uso dello Scheduler**:
> - Non bloccare la UI quando elabori grandi quantità di dati
> - Controllo tempo nei test (TestScheduler)
> - Controllo event loop in ambiente Node.js

Per maggiori informazioni, consulta [Tipi di Scheduler e Come Usarli](/it/guide/schedulers/types).

## Confronto con Altre Funzioni di Creazione

### range() vs of()

```typescript
import { range, of } from 'rxjs';

// range() - interi consecutivi
range(1, 3).subscribe(console.log);
// Output: 1, 2, 3

// of() - enumera valori arbitrari
of(1, 2, 3).subscribe(console.log);
// Output: 1, 2, 3

// Differenza: range() accetta solo numeri sequenziali, of() accetta valori arbitrari
of(1, 10, 100).subscribe(console.log);
// Output: 1, 10, 100 (non possibile con range())
```

### range() vs from()

```typescript
import { range, from } from 'rxjs';

// range() - genera numeri sequenziali
range(1, 5).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// from() - genera da un array (deve creare array in anticipo)
from([1, 2, 3, 4, 5]).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// Vantaggio di range(): nessuna pre-allocazione di array in memoria
range(1, 1000000); // Efficiente in memoria
from(Array.from({ length: 1000000 }, (_, i) => i + 1)); // Array va in memoria
```

### range() vs generate()

```typescript
import { range, generate } from 'rxjs';

// range() - semplice numerazione sequenziale
range(1, 5).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// generate() - esempio complesso della stessa cosa
generate(
  1,                    // Valore iniziale
  x => x <= 5,          // Condizione continuazione
  x => x + 1            // Iterazione
).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// Vantaggi di generate(): condizione complessa e gestione stato
generate(
  1,
  x => x <= 100,
  x => x * 2  // Incrementa per fattore 2
).subscribe(console.log);
// Output: 1, 2, 4, 8, 16, 32, 64
// (non possibile con range())
```

> [!TIP]
> **Criteri di Selezione**:
> - **Servono numeri sequenziali** → `range()`
> - **Enumera valori arbitrari** → `of()`
> - **Array/Promise esistente** → `from()`
> - **Condizione/step complesso** → `generate()`

## Considerazioni sulle Performance

Poiché `range()` emette valori sincronamente, le performance dovrebbero essere considerate quando si generano grandi numeri di valori.

> [!WARNING]
> **Gestione Grandi Quantità di Dati**:
> ```typescript
> // ❌ Cattivo esempio: emette 1 milione di valori sincronamente (UI si blocca)
> range(1, 1000000).subscribe(console.log);
>
> // ✅ Buon esempio 1: asincrono con scheduler
> range(1, 1000000).pipe(
>   observeOn(asyncScheduler)
> ).subscribe(console.log);
>
> // ✅ Buon Esempio 2: Dividi con buffering
> range(1, 1000000).pipe(
>   bufferCount(1000)
> ).subscribe(batch => console.log(`${batch.length} casi elaborati`));
> ```

## Gestione Errori

Sebbene `range()` stesso non emetta errori, errori possono verificarsi nella pipeline.

```typescript
import { range, of, map, catchError } from 'rxjs';
range(1, 10).pipe(
  map(n => {
    if (n === 5) {
      throw new Error('Errore a 5');
    }
    return n * 2;
  }),
  catchError(error => {
    console.error('Si è verificato un errore:', error.message);
    return of(-1); // Ritorna valore default
  })
).subscribe(console.log);

// Output: 2, 4, 6, 8, -1
```

## Riepilogo

`range()` è una Funzione di Creazione semplice ma potente che produce una sequenza di interi consecutivi.

> [!IMPORTANT]
> **Caratteristiche di range()**:
> - ✅ Ideale per generare numeri consecutivi (alternativa a istruzione for)
> - ✅ Utile per elaborazione batch, paginazione, generazione dati test
> - ✅ Efficiente in memoria (nessuna pre-creazione di array)
> - ⚠️ Considera asincrono per grandi quantità di dati
> - ⚠️ Usa `generate()` per condizioni complesse

## Argomenti Correlati

- [generate()](/it/guide/creation-functions/loop/generate) - Generazione loop generica
- [of()](/it/guide/creation-functions/basic/of) - Enumera valori arbitrari
- [from()](/it/guide/creation-functions/basic/from) - Converti da array o Promise
- [interval()](/it/guide/creation-functions/basic/interval) - Pubblica valori periodicamente

## Riferimenti

- [RxJS Ufficiale: range()](https://rxjs.dev/api/index/function/range)
- [Learn RxJS: range](https://www.learnrxjs.io/learn-rxjs/operators/creation/range)
