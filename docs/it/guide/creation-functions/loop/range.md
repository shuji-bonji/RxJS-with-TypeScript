---
description: "range() - Creation Function che emette numeri interi consecutivi in sequenza a partire da un valore di partenza specificato; un metodo dichiarativo di generazione di numeri sequenziali come alternativa all'istruzione for; come viene utilizzato con generate(), la sua implementazione utilizzando l'inferenza di tipo in TypeScript e il suo schema di combinazione con map(). Spiegato con esempi pratici di codice."
---

# range() - Genera un range numerico

range()` è una Creation Function simile a un for statement che genera un numero specifico di numeri interi consecutivi a partire da un valore iniziale specificato.

## Panoramica.

range()` emette una sequenza di numeri interi consecutivi come Observable, specificando un valore iniziale e il numero di numeri interi. Viene utilizzata per la generazione di numeri sequenziali e per l'elaborazione in batch, come alternativa dichiarativa alla tradizionale istruzione `for`.

**Firma**:.

```typescript
function range(
  start: number,
  count?: number,
  scheduler?: SchedulerLike
): Observable<number>
```

**Parametri**:.
- `start`: il valore iniziale (l'emissione inizia da questo valore).
- `count`: numero di valori da pubblicare (omesso, da 0 a meno di `start`).
- `scheduler`: Scheduler per emettere i valori (omesso, emessi in modo sincrono).

**Documentazione ufficiale**: [📘 Formula RxJS: range()](https://rxjs.dev/api/index/function/range)

## Utilizzo di base.

### Schema 1: Specificare il valore iniziale e il numero di pezzi

Uso più comune.

```typescript
import { range } from 'rxjs';

// 1da5Vengono generati numeri sequenziali (1, 2, 3, 4, 5)
range(1, 5).subscribe({
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

### Schema 2: Numeri sequenziali a partire da 0

Impostando il valore iniziale a 0, è possibile generare un numero sequenziale come indice di un array.

```typescript
import { range } from 'rxjs';

// 0da10Generazione di numeri sequenziali (0, 1, 2, ..., 9)
range(0, 10).subscribe(console.log);
// Uscita: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
```

### Schema 3: Iniziare con un numero negativo

Può essere generato anche da numeri negativi.

```typescript
import { range } from 'rxjs';

// -3da5Generazione di numeri sequenziali (-3, -2, -1, 0, 1)
range(-3, 5).subscribe(console.log);
// Uscita: -3, -2, -1, 0, 1
```

## Caratteristiche principali.

### 1. emesso in modo sincrono

range()` emette tutti i valori in modo **sincrono** con sottoscrizione per impostazione predefinita.

```typescript
import { range } from 'rxjs';

console.log('Prima della sottoscrizione');

range(1, 3).subscribe(value => console.log('Valore:', value));

console.log('Dopo la sottoscrizione');

// Uscita:
// Prima della sottoscrizione
// Valore: 1
// Valore: 2
// Valore: 3
// Dopo la sottoscrizione
```

### 2. completamento immediato.

Notifica il completamento immediatamente dopo che tutti i valori sono stati emessi.

```typescript
import { range } from 'rxjs';

range(1, 3).subscribe({
  next: val => console.log(val),
  complete: () => console.log('Completato！')
});

// Uscita: 1, 2, 3, Completato！
```

### 3. equivalenza con l'istruzione for

range(start, count)` è equivalente alla seguente istruzione for.

```typescript
// ImperativoforFrase
for (let i = start; i < start + count; i++) {
  console.log(i);
}

// Dichiarativorange()
range(start, count).subscribe(console.log);
```

## Caso d'uso pratico.

### 1. elaborazione in batch

Utilizzata per l'esecuzione sequenziale di più attività.

```typescript
import { range, of, Observable, concatMap, delay, map } from 'rxjs';
// Funzioni che simulano l'elaborazione dei dati
function processItem(index: number): Observable<string> {
  return of(index).pipe(
    delay(100), // 100msSimulare i tempi di elaborazione per
    map(i => `elemento${i}Risultati dell'elaborazione di`)
  );
}

// 10Elaborazione sequenziale dei dati per un elemento (con un ritardo di1secondi di ritardo tra un'elaborazione e l'altra)
range(1, 10).pipe(
  concatMap(index =>
    processItem(index).pipe(delay(1000))
  )
).subscribe({
  next: result => console.log(`Elaborazione completata: ${result}`),
  complete: () => console.log('Tutte le elaborazioni sono state completate')
});

// Uscita:
// Elaborazione completata: elemento1Risultati dell'elaborazione di (Dopo circa1.1secondi dopo)
// Elaborazione completata: elemento2Risultati dell'elaborazione di (Dopo circa2.1secondi dopo)
// ...
// Elaborazione completata: elemento10Risultati dell'elaborazione di (Dopo circa10.1secondi dopo)
// Tutte le elaborazioni sono state completate
```

### 2. paginazione

Recupera i dati di più pagine in modo sequenziale.

```typescript
import { range, of, Observable, concatMap, delay } from 'rxjs';
interface PageData {
  page: number;
  items: string[];
}

// Funzione per simulare l'acquisizione dei dati della pagina
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`elemento${page}-1`, `elemento${page}-2`, `elemento${page}-3`]
  }).pipe(
    delay(500) // APISimulare la chiamata
  );
}

function fetchAllPages(totalPages: number) {
  return range(1, totalPages).pipe(
    concatMap(page => fetchPage(page))
  );
}

fetchAllPages(5).subscribe({
  next: (data: PageData) => console.log(`Pagina ${data.page}:`, data.items),
  complete: () => console.log('Completamento di tutte le acquisizioni di pagine')
});

// Uscita:
// Pagina 1: ['elemento1-1', 'elemento1-2', 'elemento1-3']
// Pagina 2: ['elemento2-1', 'elemento2-2', 'elemento2-3']
// Pagina 3: ['elemento3-1', 'elemento3-2', 'elemento3-3']
// Pagina 4: ['elemento4-1', 'elemento4-2', 'elemento4-3']
// Pagina 5: ['elemento5-1', 'elemento5-2', 'elemento5-3']
// Completamento di tutte le acquisizioni di pagine
```

### 3. Elaborare gli indici degli array

Utilizzato come ciclo basato sull'indice per l'elaborazione di ciascun elemento di un array.

```typescript
function range(
  start: number,
  count?: number,
  scheduler?: SchedulerLike
): Observable<number>
```

### 4. Generare i dati di prova

È utile per generare dati finti nei test unitari.


### 5. Contatore del processo di ripetizione

Controlla il numero di tentativi in caso di errori.


## Asincronizzazione tramite scheduler

Quando si elaborano grandi quantità di dati, è possibile specificare uno scheduler per l'esecuzione asincrona.


> [!TIP]

> **Utilizzo dello scheduler**:.
> - Non bloccare l'interfaccia utente con l'elaborazione di dati di massa.
> - Controllo del tempo nei test (Scheduler)
> Controllo dei cicli di eventi in ambienti Node.js

Per ulteriori informazioni, vedere [Tipi di scheduler e loro utilizzo] (/it/guide/schedulers/types).

## Confronto con altre Creation Function

### range() vs of()


```typescript
function range(
  start: number,
  count?: number,
  scheduler?: SchedulerLike
): Observable<number>
```

## range() vs from()


## range() vs generate()


```typescript
import { range } from 'rxjs';

// 1da5Vengono generati numeri sequenziali (1, 2, 3, 4, 5)
range(1, 5).subscribe({
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

> **Criteri di selezione**:.
> - **Richiede un numero sequenziale** → `range()`
> - **Enumera un valore qualsiasi** → `of()`
> - **Promise esistente** → `from()`
> - **Condizione complessa/passo** → `generare()`

## Note sulle prestazioni

range()` emette i valori in modo sincrono, quindi è necessario prendere nota delle prestazioni quando si generano un gran numero di valori.

> [!WARNING]

> **Gestione di grandi quantità di dati**:.
>


```typescript
function range(
  start: number,
  count?: number,
  scheduler?: SchedulerLike
): Observable<number>
```

dattiloscritto
importare { range, from } da 'rxjs';.

// se sono richiesti numeri sequenziali → range() è conciso
range(0, 10).subscribe(console.log);

// Non è necessario creare un array e poi convertirlo (inefficiente)
from(Array.from({ length: 10 }, (_, i) => i)).subscribe(console.log);

// Se esiste un array esistente → usare from()
const existingArray = [5, 10, 15, 20];
from(existingArray).subscribe(console.log);


```typescript
function range(
  start: number,
  count?: number,
  scheduler?: SchedulerLike
): Observable<number>
```

dattiloscritto.
import { range, of, map, catchError } from 'rxjs';
range(1, 10).pipe(
  map(n => {
    se (n === 5) {
      lancia un nuovo errore('Errore a 5');
    }
    return n * 2;
  }),.
  catchError((error: unknown) => {
    const message = error of Error? error.message : String(error);
    console.error('Error occurred:', message);
    return of(-1); // restituisce il valore predefinito
  })
).subscribe(console.log);.

// Output: 2, 4, 6, 8, -1


## Riepilogo

range()` è una semplice e potente Creation Function che produce una sequenza di numeri interi consecutivi.

> [!IMPORTANT]

> **Caratteristiche di range()`**:.
> ✅ Ideale per generare numeri consecutivi (alternativa all'istruzione for).
> - ✅ Utile per l'elaborazione in batch, l'impaginazione e la generazione di dati di prova.
> - ✅ Efficiente dal punto di vista della memoria (nessuna creazione preliminare di array).
> - ⚠️ Considerare l'asincronia per grandi quantità di dati
> - ⚠️ Usare `generate()` per condizioni complesse

## Vedere anche.

- [generate()](/it/guide/creation-functions/loop/generate) - Generica generazione di loop.
- [of()](/it/guide/creation-functions/basic/of) - enumera valori arbitrari.
- [from()](/it/guide/creation-functions/basic/from) - Converte da un array o da Promise
- [interval()](/it/guide/creation-functions/basic/interval) - Pubblica valori periodicamente

## Risorsa di riferimento.

- [RxJS official: range()](https://rxjs.dev/api/index/function/range)
- [Imparare RxJS: range](https://www.learnrxjs.io/learn-rxjs/operators/creation/range)
