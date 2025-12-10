---
description: "generate() - Generazione loop generica con controllo condizione flessibile: Loop dichiarativi simili a while per Fibonacci, paginazione e gestione stato personalizzata"
---

# generate() - Generazione Loop Generica

`generate()` è una Funzione di Creazione che fornisce elaborazione loop flessibile come Observable specificando stato iniziale, condizione continuazione, aggiornamento stato e selezione risultato.

## Panoramica

`generate()` può descrivere dichiarativamente elaborazione loop flessibile come istruzioni while e for. Viene usata quando è richiesta condizione o gestione stato più complessa di `range()`.

**Firma**:
```typescript
function generate<T, S>(
  initialState: S,
  condition: (state: S) => boolean,
  iterate: (state: S) => S,
  resultSelector?: (state: S) => T,
  scheduler?: SchedulerLike
): Observable<T>
```

**Parametri**:
- `initialState`: Lo stato iniziale del loop
- `condition`: Funzione per determinare la condizione continuazione (`false` termina il loop)
- `iterate`: Funzione per avanzare lo stato al prossimo (aggiorna stato)
- `resultSelector`: Funzione per selezionare un valore da emettere dallo stato (se omesso, emette lo stato stesso)
- `scheduler`: Scheduler che emette valori (se omesso: emette valori sincronamente)

**Documentazione Ufficiale**: [📘 RxJS Ufficiale: generate()](https://rxjs.dev/api/index/function/generate)

## Uso Base

### Pattern 1: Contatore Semplice

Questo è l'utilizzo più base.

```typescript
import { generate } from 'rxjs';

// Conta da 1 a 5
generate(
  1,              // Stato iniziale
  x => x <= 5,    // Condizione continuazione
  x => x + 1      // Aggiornamento stato
).subscribe({
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

Questo codice è equivalente alla seguente istruzione while:

```typescript
let x = 1;
while (x <= 5) {
  console.log('Valore:', x);
  x = x + 1;
}
console.log('Completo');
```

### Pattern 2: Converti Valori con resultSelector

Puoi separare lo stato dal valore da emettere.

```typescript
import { generate } from 'rxjs';

// Lo stato interno è un contatore, ma il valore emesso è un valore al quadrato
generate(
  1,              // Stato iniziale: 1
  x => x <= 5,    // Condizione continuazione: x <= 5
  x => x + 1,     // Aggiornamento stato: x + 1
  x => x * x      // Selezione risultato: emetti x^2
).subscribe(console.log);

// Output: 1, 4, 9, 16, 25
```

### Pattern 3: Oggetto Stato Complesso

Oggetti complessi possono essere usati come stati.

```typescript
import { generate } from 'rxjs';

interface State {
  count: number;
  sum: number;
}

// Calcola somma cumulativa
generate<number, State>(
  { count: 1, sum: 0 },           // Stato iniziale
  state => state.count <= 5,      // Condizione continuazione
  state => ({                     // Aggiornamento stato
    count: state.count + 1,
    sum: state.sum + state.count
  }),
  state => state.sum              // Selezione risultato
).subscribe(console.log);

// Output: 0, 1, 3, 6, 10
// (0, 0+1, 0+1+2, 0+1+2+3, 0+1+2+3+4)
```

## Caratteristiche Importanti

### 1. Comportamento Simile a Istruzione While

`generate()` fornisce controllo flessibile come un'istruzione while.

```typescript
import { generate } from "rxjs";

// Istruzione while
let i = 1;
while (i <= 10) {
  console.log(i);
  i = i * 2;
}

// Stessa cosa con generate()
generate(
  1,              // let i = 1;
  i => i <= 10,   // while (i <= 10)
  i => i * 2      // i = i * 2;
).subscribe(console.log);

// Output: 1, 2, 4, 8
```

### 2. Emissione Sincrona

Per default, tutti i valori vengono pubblicati **sincronamente** alla subscription.

```typescript
import { generate } from 'rxjs';

console.log('Prima della subscription');

generate(1, x => x <= 3, x => x + 1).subscribe(val => console.log('Valore:', val));

console.log('Dopo la subscription');

// Output:
// Prima della subscription
// Valore: 1
// Valore: 2
// Valore: 3
// Dopo la subscription
```

### 3. Attenzione ai Loop Infiniti

Se la condizione è sempre `true`, otterrai un loop infinito.

```typescript
import { generate, take } from 'rxjs';
// ❌ Pericolo: loop infinito (browser si blocca)
// generate(0, x => true, x => x + 1).subscribe(console.log);

// ✅ Sicuro: usa take() per limitare numero
generate(
  0,
  x => true,  // Sempre true
  x => x + 1
).pipe(
  take(10)    // Ottieni solo i primi 10
).subscribe(console.log);

// Output: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
```

> [!WARNING]
> **Attenzione ai Loop Infiniti**:
> - Se la condizione è sempre `true`, si verifica un loop infinito
> - Usa `take()`, `takeWhile()`, o `takeUntil()` per limitare il numero di emissioni
> - Oppure imposta condizioni di uscita appropriate con funzioni condizionali

## Casi d'Uso Pratici

### 1. Sequenza Fibonacci

Esempio di transizioni di stato complesse.

```typescript
import { generate, take } from 'rxjs';
interface FibState {
  current: number;
  next: number;
}

// Primi 10 termini della sequenza Fibonacci
generate<number, FibState>(
  { current: 0, next: 1 },           // Stato iniziale: F(0)=0, F(1)=1
  state => true,                     // Generato infinitamente
  state => ({                        // Aggiornamento stato
    current: state.next,
    next: state.current + state.next
  }),
  state => state.current             // Emetti valore corrente
).pipe(
  take(10)                           // Primi 10 termini
).subscribe(console.log);

// Output: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

### 2. Backoff Esponenziale

Questa è la generazione di tempo di attesa esponenziale usata nel processo di retry.

```typescript
import { generate } from 'rxjs';

interface RetryState {
  attempt: number;
  delay: number;
}

// Genera delay per backoff esponenziale (1, 2, 4, 8, 16 secondi)
generate<number, RetryState>(
  { attempt: 0, delay: 1000 },       // Stato iniziale: 1 secondo
  state => state.attempt < 5,        // Massimo 5 tentativi
  state => ({                        // Aggiornamento stato
    attempt: state.attempt + 1,
    delay: state.delay * 2           // Raddoppia il tempo di delay
  }),
  state => state.delay               // Emetti tempo di delay
).subscribe(delay => {
  console.log(`Retry tra ${delay / 1000} secondi`);
});

// Output:
// Retry tra 1 secondo
// Retry tra 2 secondi
// Retry tra 4 secondi
// Retry tra 8 secondi
// Retry tra 16 secondi
```

### 3. Controllo Paginazione

Continua a recuperare finché esiste una pagina successiva.

```typescript
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

// Funzione per simulare recupero dati pagina
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`Elemento${page}-1`, `Elemento${page}-2`, `Elemento${page}-3`],
    hasNext: page < 10 // Fino a pagina 10
  }).pipe(
    delay(500) // Simula chiamata API
  );
}

// Ottieni la pagina finché esiste (ottieni hasNext dalla risposta API)
generate<number, PageState>(
  { page: 1, hasNext: true },        // Stato iniziale
  state => state.hasNext,            // Continua finché c'è pagina successiva
  state => ({                        // Aggiornamento stato
    page: state.page + 1,
    hasNext: state.page < 10         // Supponi ci siano fino a 10 pagine
  }),
  state => state.page                // Emetti numero pagina
).pipe(
  concatMap(page => fetchPage(page)) // Recupera ogni pagina in ordine
).subscribe(
  data => console.log(`Pagina ${data.page} recuperata:`, data.items),
  err => console.error('Errore:', err),
  () => console.log('Tutte le pagine recuperate')
);

// Output:
// Pagina 1 recuperata: ['Elemento1-1', 'Elemento1-2', 'Elemento1-3']
// Pagina 2 recuperata: ['Elemento2-1', 'Elemento2-2', 'Elemento2-3']
// ...
// Pagina 10 recuperata: ['Elemento10-1', 'Elemento10-2', 'Elemento10-3']
// Tutte le pagine recuperate
```

### 4. Calcolo Fattoriale

Rappresenta calcoli matematici come stream.

```typescript
import { generate } from 'rxjs';

interface FactorialState {
  n: number;
  result: number;
}

// Calcola fattoriale di 5 (5! = 5 × 4 × 3 × 2 × 1 = 120)
generate<number, FactorialState>(
  { n: 5, result: 1 },               // Stato iniziale
  state => state.n > 0,              // Continua per n > 0
  state => ({                        // Aggiornamento stato
    n: state.n - 1,
    result: state.result * state.n
  }),
  state => state.result              // Emetti risultato intermedio
).subscribe(console.log);

// Output: 5, 20, 60, 120, 120
// (1*5, 5*4, 20*3, 60*2, 120*1)
```

## Confronto con Altre Funzioni di Creazione

### generate() vs range()

```typescript
import { generate, range } from 'rxjs';

// range() - semplice numerazione sequenziale
range(1, 5).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// generate() - stessa cosa, ma più esplicito
generate(
  1,
  x => x <= 5,
  x => x + 1
).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// Vero valore di generate(): step complessi
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
> - **Numeri sequenziali semplici** → `range()`
> - **Condizioni o step complessi** → `generate()`
> - **Determinato dinamicamente alla subscription** → `defer()`
> - **Fibonacci, fattoriale, ecc.** → `generate()`

## Considerazioni sulle Performance

Poiché `generate()` emette valori sincronamente, le performance dovrebbero essere considerate quando si generano grandi numeri di valori o si eseguono calcoli complessi.

> [!WARNING]
> **Ottimizzazione Performance**:
> ```typescript
> // ❌ Cattivo esempio: calcolo complesso eseguito sincronamente (UI blocca)
> generate(
>   1,
>   x => x <= 1000000,
>   x => expensiveCalculation(x)
> ).subscribe(console.log);
>
> // ✅ Buon esempio 1: asincrono con scheduler
> generate(
>   1,
>   x => x <= 1000000,
>   x => expensiveCalculation(x)
> ).pipe(
>   observeOn(asyncScheduler)
> ).subscribe(console.log);
>
> // ✅ Buon Esempio 2: Limita numero con take()
> generate(
>   1,
>   x => true,  // Loop infinito
>   x => x + 1
> ).pipe(
>   take(100)   // Solo i primi 100
> ).subscribe(console.log);
> ```

## Riepilogo

`generate()` è una potente Funzione di Creazione che permette di descrivere dichiarativamente elaborazione loop complessa.

> [!IMPORTANT]
> **Caratteristiche di generate()**:
> - ✅ Controllo loop flessibile come istruzioni while/for
> - ✅ Gestione stato complessa possibile
> - ✅ Ideale per calcoli matematici come Fibonacci, fattoriale, ecc.
> - ✅ Stato e valori emessi possono essere separati
> - ⚠️ Attenzione ai loop infiniti (limitati da `take()`)
> - ⚠️ Considera asincrono per grandi quantità di dati
> - ⚠️ Usa `range()` per numeri sequenziali semplici

## Argomenti Correlati

- [range()](/it/guide/creation-functions/loop/range) - Semplice generazione numeri sequenziali
- [defer()](/it/guide/creation-functions/conditional/defer) - Generazione dinamica alla subscription
- [expand()](/it/guide/operators/transformation/expand) - Espansione ricorsiva (operatore higher-order)
- [scan()](/it/guide/operators/transformation/scan) - Calcolo cumulativo

## Riferimenti

- [RxJS Ufficiale: generate()](https://rxjs.dev/api/index/function/generate)
- [Learn RxJS: generate](https://www.learnrxjs.io/learn-rxjs/operators/creation/generate)
