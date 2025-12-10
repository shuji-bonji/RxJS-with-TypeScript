---
description: "of() - Creation Function RxJS che emette i valori specificati in ordine. Il metodo più semplice per creare Observable, ottimale per la creazione di dati di test e mock. Spiega pattern di implementazione utilizzando l'inferenza di tipo TypeScript, come gestire array e oggetti, e come distinguerlo da from()."
---

# of() - Emissione sequenziale di valori

`of()` è la Creation Function più semplice che emette uno per uno in ordine i valori specificati.

## Panoramica

`of()` emette in ordine i valori passati come argomenti immediatamente dopo la sottoscrizione, e completa immediatamente dopo aver emesso tutti i valori. Viene utilizzato frequentemente per la creazione di codice di test e dati mock.

**Signature**:
```typescript
function of<T>(...args: T[]): Observable<T>
```

**Documentazione ufficiale**: [📘 RxJS Official: of()](https://rxjs.dev/api/index/function/of)

## Utilizzo basilare

`of()` può ricevere più valori separati da virgola.

```typescript
import { of } from 'rxjs';

const values$ = of(1, 2, 3, 4, 5);

values$.subscribe({
  next: value => console.log('Valore:', value),
  error: err => console.error('Errore:', err),
  complete: () => console.log('Completato')
});

// Output:
// Valore: 1
// Valore: 2
// Valore: 3
// Valore: 4
// Valore: 5
// Completato
```

## Caratteristiche importanti

### 1. Emissione sincrona

`of()` emette tutti i valori **sincronamente** immediatamente dopo la sottoscrizione.

```typescript
import { of } from 'rxjs';

console.log('Prima della sottoscrizione');

of('A', 'B', 'C').subscribe(value => console.log('Valore:', value));

console.log('Dopo la sottoscrizione');

// Output:
// Prima della sottoscrizione
// Valore: A
// Valore: B
// Valore: C
// Dopo la sottoscrizione
```

### 2. Completamento immediato

Dopo aver emesso tutti i valori, notifica immediatamente `complete`.

```typescript
import { of } from 'rxjs';

of(1, 2, 3).subscribe({
  next: val => console.log(val),
  complete: () => console.log('Completato!')
});

// Output: 1, 2, 3, Completato!
```

### 3. Può emettere valori di qualsiasi tipo

Può emettere valori di qualsiasi tipo, da tipi primitivi a oggetti e array.

```typescript
import { of } from 'rxjs';

// Tipi primitivi
of(42, 'hello', true).subscribe(console.log);

// Oggetti
of(
  { id: 1, name: 'Alice' },
  { id: 2, name: 'Bob' }
).subscribe(console.log);

// Array (emette l'array stesso come un valore)
of([1, 2, 3], [4, 5, 6]).subscribe(console.log);
// Output: [1, 2, 3], [4, 5, 6]
```

### 4. Cold Observable

`of()` è un **Cold Observable**. Ad ogni sottoscrizione inizia un'esecuzione indipendente.

```typescript
import { of } from 'rxjs';

const values$ = of(1, 2, 3);

// Prima sottoscrizione
values$.subscribe(val => console.log('Sottoscrittore A:', val));

// Seconda sottoscrizione (eseguita indipendentemente)
values$.subscribe(val => console.log('Sottoscrittore B:', val));

// Output:
// Sottoscrittore A: 1
// Sottoscrittore A: 2
// Sottoscrittore A: 3
// Sottoscrittore B: 1
// Sottoscrittore B: 2
// Sottoscrittore B: 3
```

> [!NOTE]
> **Caratteristiche dei Cold Observable**
> - Ad ogni sottoscrizione inizia un'esecuzione indipendente
> - Ogni sottoscrittore riceve il proprio data stream
> - Se è necessaria la condivisione dei dati, è necessario Hot-izzare con `share()`, ecc.
>
> Per maggiori dettagli, consultare [Cold Observable e Hot Observable](/it/guide/observables/cold-and-hot-observables).

## Differenza tra of() e from()

`of()` e `from()` si comportano diversamente quando si gestiscono array. Questo è un punto di confusione comune.

```typescript
import { of, from } from 'rxjs';

// of() - Emette l'array come un unico valore
of([1, 2, 3]).subscribe(console.log);
// Output: [1, 2, 3]

// from() - Emette individualmente ogni elemento dell'array
from([1, 2, 3]).subscribe(console.log);
// Output: 1, 2, 3
```

> [!IMPORTANT]
> **Criteri di scelta**:
> - Si vuole emettere l'array stesso → `of([1, 2, 3])`
> - Si vuole emettere individualmente ogni elemento dell'array → `from([1, 2, 3])`

## Casi d'uso pratici

### 1. Creazione di dati di test e mock

`of()` viene utilizzato più frequentemente per creare dati mock nel codice di test.

```typescript
import { of } from 'rxjs';

// Mock di dati utente
function getMockUser$() {
  return of({
    id: 1,
    name: 'Test User',
    email: 'test@example.com'
  });
}

// Utilizzo nel test
getMockUser$().subscribe(user => {
  console.log('User:', user.name); // User: Test User
});
```

### 2. Fornitura di valore predefinito

Viene utilizzato per fornire valori di fallback in caso di errore o valori predefiniti.

```typescript
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

function fetchData(id: number) {
  if (id < 0) {
    return throwError(() => new Error('Invalid ID'));
  }
  return of({ id, data: 'some data' });
}

fetchData(-1).pipe(
  catchError(err => {
    console.error('Errore:', err.message);
    return of({ id: 0, data: 'default data' }); // Valore predefinito
  })
).subscribe(result => console.log(result));
// Output: Errore: Invalid ID
//       { id: 0, data: 'default data' }
```

### 3. Emissione graduale di più valori

Viene utilizzato per eseguire più passaggi in ordine.

```typescript
import { of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

of('Loading...', 'Processing...', 'Done!').pipe(
  concatMap(message => of(message).pipe(delay(1000)))
).subscribe(console.log);

// Output (ogni secondo):
// Loading...
// Processing...
// Done!
```

### 4. Restituzione di valori con diramazione condizionale

Viene utilizzato in combinazione con `iif()` o `switchMap()` per restituire valori in base a condizioni.

```typescript
import { of, iif } from 'rxjs';

const isAuthenticated = true;

iif(
  () => isAuthenticated,
  of('Welcome back!'),
  of('Please log in')
).subscribe(console.log);
// Output: Welcome back!
```

## Utilizzo nella pipeline

`of()` viene utilizzato come punto di partenza della pipeline o per iniettare dati a metà.

```typescript
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

of(1, 2, 3, 4, 5).pipe(
  filter(n => n % 2 === 0),  // Solo pari
  map(n => n * 10)           // Moltiplica per 10
).subscribe(console.log);
// Output: 20, 40
```

## Errori comuni

### 1. Passare l'array così com'è

```typescript
// ❌ Sbagliato - L'intero array viene emesso come un unico valore
of([1, 2, 3]).subscribe(console.log);
// Output: [1, 2, 3]

// ✅ Corretto - Usare from() per emettere individualmente ogni elemento
from([1, 2, 3]).subscribe(console.log);
// Output: 1, 2, 3

// ✅ Oppure usare lo spread operator
of(...[1, 2, 3]).subscribe(console.log);
// Output: 1, 2, 3
```

### 2. Confusione con elaborazione asincrona

`of()` emette sincronamente. Non diventa un'elaborazione asincrona, quindi è necessaria attenzione.

```typescript
// ❌ Questo non diventa asincrono
of(fetchDataFromAPI()).subscribe(console.log);
// fetchDataFromAPI() viene eseguito immediatamente e l'oggetto Promise viene emesso

// ✅ Usare from() per streamificare una Promise
from(fetchDataFromAPI()).subscribe(console.log);
```

## Considerazioni sulle prestazioni

`of()` è molto leggero e ha un overhead di prestazioni quasi trascurabile. Tuttavia, quando si emettono grandi quantità di valori, prestare attenzione ai seguenti punti.

> [!TIP]
> Quando si emettono grandi quantità di valori (diverse migliaia o più) in sequenza, considerare l'utilizzo di `from()` o `range()`.

## Creation Functions correlate

| Function | Differenza | Scelta |
|----------|------|----------|
| **[from()](/it/guide/creation-functions/basic/from)** | Converte da array o Promise | Streamificare iterabili o Promise |
| **range()** | Genera un intervallo di numeri | Emettere numeri consecutivi |
| **EMPTY** | Completa immediatamente senza emettere nulla | Quando è necessario uno stream vuoto |

## Riepilogo

- `of()` è la Creation Function più semplice che emette i valori specificati in ordine
- Emette sincronamente immediatamente dopo la sottoscrizione e completa immediatamente
- Ottimale per la creazione di dati di test e mock
- Quando si passa un array, viene emesso l'array stesso (differente da `from()`)
- Usare `from()` per elaborazione asincrona

## Prossimi passi

- [from() - Conversione da array, Promise, ecc.](/it/guide/creation-functions/basic/from)
- [Creation Functions di Combinazione](/it/guide/creation-functions/combination/)
- [Ritorno alla panoramica della creazione di base](/it/guide/creation-functions/basic/)
