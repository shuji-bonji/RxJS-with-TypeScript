---
description: "generate() - una Creation Function generica per la generazione di loop che consente di specificare i valori iniziali, le condizioni, l'iterazione e la selezione dei risultati; controllo flessibile delle condizioni e gestione personalizzata dello stato, come gli enunciati while e for; vengono spiegate le differenze rispetto a range(), l'implementazione sicura in TypeScript, i modelli complessi di generazione delle sequenze. modelli di generazione di sequenze complesse."
---

# generate() - generazione generica di loop

Creation Function che consente l'elaborazione flessibile di loop come Observable, specificando lo stato iniziale, la condizione di continuazione, l'aggiornamento dello stato e la selezione del risultato.

## Panoramica.

generate()` permette di descrivere in modo dichiarativo l'elaborazione flessibile dei cicli, come le istruzioni while e for. Viene utilizzato quando sono richieste condizioni o gestione dello stato più complesse di quelle di `range()`.

**Firma**:.

```typescript
function generate<T, S>(
  initialState: S,
  condition: (state: S) => boolean,
  iterate: (state: S) => S,
  resultSelector?: (state: S) => T,
  scheduler?: SchedulerLike
): Observable<T>
```

**Parametri**:.
- `initialState`: stato iniziale del ciclo.
- `condizione`: funzione per determinare la condizione di continuazione (il ciclo termina con `false`).
- `iterate`: funzione per far avanzare lo stato al successivo (aggiornamento dello stato).
- `resultSelector`: funzione per selezionare un valore da pubblicare da uno stato (se omesso, viene pubblicato lo stato stesso).
- `scheduler`: scheduler che pubblica il valore (se omesso, pubblica in modo sincrono).

**Documentazione ufficiale**: [📘 RxJS official: generate()](https://rxjs.dev/api/index/function/generate)

## Utilizzo di base.

### Schema 1: contatore semplice

Questo è l'uso più elementare.

```typescript
import { generate } from 'rxjs';

// 1da5Conteggio da
generate(
  1,              // Condizione iniziale
  x => x <= 5,    // Condizione di continuazione
  x => x + 1      // Aggiornamento dello stato
).subscribe({
  next: value => console.log('Valore:', value),
  complete: () => console.log('Completato')
});

// Uscita:
// Valore: 1
// Valore: 2
// Valore: 3
// Valore: 4
// Valore: 5
// Completato
```

Questo codice è equivalente alla seguente istruzione while.

```typescript
let x = 1;
while (x <= 5) {
  console.log('Valore:', x);
  x = x + 1;
}
console.log('Completato');
```

### Pattern 2: Conversione di valori con resultSelector

Lo stato e il valore da emettere possono essere separati.

```typescript
import { generate } from 'rxjs';

// Lo stato interno è un contatore, ma emette2Valore moltiplicato
generate(
  1,              // Condizione iniziale: 1
  x => x <= 5,    // Condizione di continuazione: x <= 5
  x => x + 1,     // Aggiornamento dello stato: x + 1
  x => x * x      // Selezione del risultato: x^2Problema
).subscribe(console.log);

// Uscita: 1, 4, 9, 16, 25
```

### Pattern 3: Oggetti di stato complessi

Gli oggetti complessi possono essere usati come stati.

```typescript
import { generate } from 'rxjs';

interface State {
  count: number;
  sum: number;
}

// Calcolo della somma cumulativa
generate<number, State>(
  { count: 1, sum: 0 },           // Condizione iniziale
  state => state.count <= 5,      // Condizione di continuazione
  state => ({                     // Aggiornamento dello stato
    count: state.count + 1,
    sum: state.sum + state.count
  }),
  state => state.sum              // Selezione del risultato
).subscribe(console.log);

// Uscita: 0, 1, 3, 6, 10
// (0, 0+1, 0+1+2, 0+1+2+3, 0+1+2+3+4)
```

## Caratteristiche importanti.

### 1. Comportamento simile alla dichiarazione while

generate()` fornisce un controllo flessibile come un'istruzione while.

```typescript
import { generate } from "rxjs";

// whiledichiarazione
let i = 1;
while (i <= 10) {
  console.log(i);
  i = i * 2;
}

// generate()Ottenere lo stesso in
generate(
  1,              // let i = 1;
  i => i <= 10,   // while (i <= 10)
  i => i * 2      // i = i * 2;
).subscribe(console.log);

// Uscita: 1, 2, 4, 8
```

### 2. Emissione sincrona

Per impostazione predefinita, tutti i valori sono pubblicati in modo **sincrono** al momento della sottoscrizione.

```typescript
import { generate } from 'rxjs';

console.log('Prima della sottoscrizione');

generate(1, x => x <= 3, x => x + 1).subscribe(val => console.log('Valore:', val));

console.log('Dopo la sottoscrizione');

// Uscita:
// Prima della sottoscrizione
// Valore: 1
// Valore: 2
// Valore: 3
// Dopo la sottoscrizione
```

### 3. Attenzione ai loop infiniti.

Se la condizione è sempre `vero', c'è un ciclo infinito.

```typescript
import { generate, take } from 'rxjs';
// ❌ Pericolo: Ciclo infinito (il browser si blocca)
// generate(0, x => true, x => x + 1).subscribe(console.log);

// ✅ Sicuro: take()Limitare il numero di pezzi a
generate(
  0,
  x => true,  // Sempretrue
  x => x + 1
).pipe(
  take(10)    // Recupera sempre il primo10Viene recuperato solo il primo pezzo
).subscribe(console.log);

// Uscita: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
```

> [!WARNING]

> **Attenzione ai loop infiniti**:.
> - Se la condizione è sempre `vero`, si verifica un ciclo infinito.
> - Limitare il numero di numeri con `take()`, `takeWhile()` e `takeUntil()`.
> oppure impostare una condizione di uscita adeguata con la funzione condition.

## Casi d'uso pratici

### 1. Sequenza di Fibonacci

Esempi di transizioni di stato complesse.

```typescript
import { generate, take } from 'rxjs';
interface FibState {
  current: number;
  next: number;
}

// Primo termine della sequenza di Fibonacci10termine della sequenza di Fibonacci
generate<number, FibState>(
  { current: 0, next: 1 },           // Condizione iniziale: F(0)=0, F(1)=1
  state => true,                     // Generato all'infinito
  state => ({                        // Aggiornamento dello stato
    current: state.next,
    next: state.current + state.next
  }),
  state => state.current             // Emettere il valore corrente
).pipe(
  take(10)                           // Recupera sempre il primo10termine della sequenza di Fibonacci
).subscribe(console.log);

// Uscita: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

### 2. Back-off esponenziale

La generazione di tempi di attesa esponenziali utilizzata nel processo di retry.

```typescript
function generate<T, S>(
  initialState: S,
  condition: (state: S) => boolean,
  iterate: (state: S) => S,
  resultSelector?: (state: S) => T,
  scheduler?: SchedulerLike
): Observable<T>
```

### 3. Controllo della paginazione.

L'acquisizione continua finché esiste la pagina successiva.

{```typescript
import { generate, of, Observable, concatMap, delay } from 'rxjs';
interface PageState {
  page: number;
  hasNext: boolean;
}

interface PageData {
  page: number;
  items: string[];
  hasNext: boolean;
}

// Funzione di simulazione dell'acquisizione dei dati della pagina
function fetchPage(page: number): Observable\<PageData> {
  return of({
    page,
    items: [`Elemento${page}-1`, `Elemento${page}-2`, `Elemento${page}-3`],
    hasNext: page < 10 // 10Fino alla pagina
  }).pipe(
    delay(500) // APISimulare la chiamata
  );
}

// Acquisito finché la pagina esiste (in pratica)APIdalla rispostahasNextdalla risposta)
generate<number, PageState>(
  { page: 1, hasNext: true },        // Condizione iniziale
  state => state.hasNext,            // Continua finché esiste la pagina successiva
  state => ({                        // Aggiornamento dello stato
    page: state.page + 1,
    hasNext: state.page < 10         // Supponiamo10Pagina fino alla pagina.
  }),
  state => state.page                // Emettere i numeri di pagina
).pipe(
  concatMap(page => fetchPage(page)) // Recupera ogni pagina a turno
).subscribe(
  data => console.log(`Pagina ${data.page} Recupera ogni pagina in sequenza:`, data.items),
  err => console.error('Errore:', err),
  () => console.log('Acquisizione di tutte le pagine completata')
);

// Uscita:
// Pagina 1 Recupera ogni pagina in sequenza: ['Elemento1-1', 'Elemento1-2', 'Elemento1-3']
// Pagina 2 Recupera ogni pagina in sequenza: ['Elemento2-1', 'Elemento2-2', 'Elemento2-3']
// ...
// Pagina 10 Recupera ogni pagina in sequenza: ['Elemento10-1', 'Elemento10-2', 'Elemento10-3']
// Acquisizione di tutte le pagine completata


### 4. timer personalizzato

Emette eventi a intervalli irregolari.

```typescript
function generate<T, S>(
  initialState: S,
  condition: (state: S) => boolean,
  iterate: (state: S) => S,
  resultSelector?: (state: S) => T,
  scheduler?: SchedulerLike
): Observable<T>
```

### 5. Calcolo del fattoriale

Rappresentare i calcoli matematici come flussi.


```typescript
function generate<T, S>(
  initialState: S,
  condition: (state: S) => boolean,
  iterate: (state: S) => S,
  resultSelector?: (state: S) => T,
  scheduler?: SchedulerLike
): Observable<T>
```

## Confronto con altre Creation Function

### generate() vs range()


```typescript
function generate<T, S>(
  initialState: S,
  condition: (state: S) => boolean,
  iterate: (state: S) => S,
  resultSelector?: (state: S) => T,
  scheduler?: SchedulerLike
): Observable<T>
```

### generate() vs defer()


```typescript
function generate<T, S>(
  initialState: S,
  condition: (state: S) => boolean,
  iterate: (state: S) => S,
  resultSelector?: (state: S) => T,
  scheduler?: SchedulerLike
): Observable<T>
```

> [!TIP]

> **Criteri di selezione**:.
> - **Numeri sequenziali semplici** → `range()`
> - **Condizioni o fasi complesse** → `generate()`
> - **Determinati dinamicamente su sottoscrizione** → `defer()`
> - **Fibonacci, fattoriale, ecc** → `generare()`

## Asincronizzazione tramite scheduler

Quando si elaborano grandi quantità di dati, è possibile specificare uno scheduler per l'esecuzione asincrona.


```typescript
function generate<T, S>(
  initialState: S,
  condition: (state: S) => boolean,
  iterate: (state: S) => S,
  resultSelector?: (state: S) => T,
  scheduler?: SchedulerLike
): Observable<T>
```

## Note sulle prestazioni

Poiché generate()` emette i valori in modo sincrono, è necessario prendere nota delle prestazioni quando si generano grandi numeri di valori o si eseguono calcoli complessi.

> [!WARNING]

> **Ottimizzazione delle prestazioni**:.
>


```typescript
function generate<T, S>(
  initialState: S,
  condition: (state: S) => boolean,
  iterate: (state: S) => S,
  resultSelector?: (state: S) => T,
  scheduler?: SchedulerLike
): Observable<T>
```

dattiloscritto.
import { generate, of, map, catchError } from 'rxjs';
generate(
  1,
  x => x <= 10,.
  x => x + 1
).pipe(
  map(n => {
    se (n === 5) {
      throw new Error('Errore a 5');)
    }
    return n * 2;
  }),.
  catchError(error => {
    console.error('Si è verificato un errore:', error.message);
    return of(-1); // restituisce il valore predefinito
  })
).subscribe(console.log);.

// Uscita: 2, 4, 6, 8, -1


```typescript
function generate<T, S>(
  initialState: S,
  condition: (state: S) => boolean,
  iterate: (state: S) => S,
  resultSelector?: (state: S) => T,
  scheduler?: SchedulerLike
): Observable<T>
```

dattiloscritto
import { generate, EMPTY, catchError } from 'rxjs';
generate(
  1,
  x => x <= 10,.
  x => {
    se (x === 5) {
      throw new Error('Errore nell'aggiornamento dello stato');
    }
    return x + 1;
  }
).pipe(
  catchError(error => {
    console.error('Error:', error.message);
    return EMPTY; // restituisce un Observable vuoto.
  })
).subscribe({
  next: console.log,.
  complete: () => console.log('Completato')
});

// Output: 1, 2, 3, 4, Error: errore nell'aggiornamento dello stato, complete


```typescript
function generate<T, S>(
  initialState: S,
  condition: (state: S) => boolean,
  iterate: (state: S) => S,
  resultSelector?: (state: S) => T,
  scheduler?: SchedulerLike
): Observable<T>
```

dattiloscritto
import { generate } from 'rxjs';.

interfaccia Stato {
  count: numero;
  sum: numero; }
}

interfaccia Result {
  indice: numero
  media: numero; }
}

// stato: State; valore emesso: Result
const stats$ = generate<Risultato, Stato>(
  { count: 1, sum: 0 }
  state => state.count <= 5,.
  stato => ({
    count: state.count + 1, state.
    sum: state.sum + state.count
  }),
  stato => ({
    indice: stato.count, stato.count
    media: state.sum / state.count
  })
);

stats$.subscribe(risultato => {
  console.log(`[${risultato.indice}] media: ${risultato.media}`);
});

// Output:.
// [1] media: 0
// [2] Media: 0,5
// [3] Media: 1
// [4] Media: 1,5
// [5] Media: 2


## Riepilogo.

Creation Function è una potente funzione che consente di descrivere in modo dichiarativo cicli complessi.

> [!IMPORTANT]

> **Caratteristiche di generate()**:.
> ✅ Controllo flessibile del ciclo alla maniera delle istruzioni while/for.
> - ✅ Possibilità di una gestione complessa degli stati.
> Ideale per calcoli matematici come Fibonacci, fattoriale, ecc.
> - ✅ Possibilità di separare i valori di stato da quelli di emissione
> - ⚠️ Attenzione ai loop infiniti (limitati da take()`)
> - ⚠️ Considerare l'asincronia per grandi quantità di dati
> - ⚠️ Usare `range()` per semplici numeri sequenziali

## Vedi anche.

- [range()](/it/guide/creation-functions/loop/range) - generazione di numeri sequenziali semplici
- defer()](/it/guide/creation-functions/conditional/defer) - generazione dinamica su sottoscrizione
- [expand()](/it/guide/operators/transformation/expand) - espansione ricorsiva (operatori di ordine superiore)
- [scan()](/it/guide/operators/transformation/scan) - Calcolo cumulativo

## Risorse di riferimento

- [RxJS official: generate()](https://rxjs.dev/api/index/function/generate)
- [Imparare RxJS: generate](https://www.learnrxjs.io/learn-rxjs/operators/creation/generate)
