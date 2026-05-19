---
description: "L'operatore filter ordina i valori in un flusso in base a una funzione condizionale specificata, lasciando passare solo i valori che soddisfano la condizione. Può essere utilizzato come funzione Type Guard (predicato di tipo) in TypeScript e spiega anche la differenza tra esso e buffer e le avvertenze per rendere una funzione predicato una funzione pura. Questa sezione spiega anche la differenza tra buffer e funzioni pure."
---

# filter - passa solo i valori che corrispondono alle condizioni

L'operatore filter ordina i valori in un flusso in base a una funzione condizionale specificata e consente di passare solo i valori che soddisfano la condizione.

## 🔰 Sintassi e uso di base

```ts
import { from } from 'rxjs';
import { filter } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);

numbers$.pipe(
  filter(n => n % 2 === 0)
).subscribe(console.log);
// Uscite: 2, 4, 6, 8, 10
```

- Vengono passati solo i valori che corrispondono alla condizione.
- Funziona in modo simile ad `Array.prototype.filter()` sugli array, ma è sequenziale su Observable.

[🌐 Documentazione ufficiale di RxJS - `filtro`](https://rxjs.dev/api/operators/filter)

## 💡 Tipico modello di utilizzo.

- Convalida dei valori di input di un modulo
- Consentire solo dati di un tipo o di una struttura specifica
- Filtraggio degli eventi del sensore e dei dati del flusso

## 🧠 Esempi pratici di codice (con UI)

Elenco in tempo reale solo se il numero inserito è pari.

```ts
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

const title = document.createElement('h3');
title.innerHTML = 'filter Esempi pratici di:';
document.body.appendChild(title);

// Creazione di campi di input
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Immissione di valori numerici';
input.style.marginBottom = '10px';
document.body.appendChild(input);

// Creare un'area di output
const output = document.createElement('div');
document.body.appendChild(output);

// Flusso di eventi in ingresso
fromEvent(input, 'input')
  .pipe(
    map((e) => parseInt((e.target as HTMLInputElement).value, 10)),
    filter((n) => !isNaN(n) && n % 2 === 0)
  )
  .subscribe((evenNumber) => {
    const item = document.createElement('div');
    item.textContent = `Rilevamento dei numeri pari: ${evenNumber}`;
    output.prepend(item);
  });

```

- Viene visualizzato nell'output solo se il numero è pari.
- Gli input dispari o non validi vengono ignorati.

> [!WARNING] 本番コードでの注意

> L'esempio precedente omette la sottoscrizione di fromEvent per semplicità di spiegazione. Nel codice reale, usare takeUntil(destroy$)`, take(N)` o `Subscription.unsubscribe()` per gestire esplicitamente il ciclo di vita. Ulteriori informazioni: [Superare le difficoltà: la gestione del ciclo di vita] (/it/guide/ Superare le difficoltà/gestione del ciclo di vita.md)

## 🔍 Differenze con il buffer

| Operatore | Operazione | Uscita. |
|---|---|---|
| Filter. | Scarta i valori che non corrispondono alla condizione. | Valori individuali `T`. |
| Buffer. | Memorizza** i valori in un array**. | Array `T[]` |

```ts
import { interval } from 'rxjs';
import { filter, buffer, take } from 'rxjs';

const source$ = interval(1000).pipe(take(5)); // 0, 1, 2, 3, 4

// filter - Vengono passati solo i valori che corrispondono alle condizioni
source$.pipe(
  filter(x => x % 2 === 0)
).subscribe(x => {
  console.log('filter:', x);
  // Uscite: filter: 0
  // Uscite: filter: 2
  // Uscite: filter: 4
});

// buffer - Memorizza i valori come array
source$.pipe(
  buffer(interval(2500))
).subscribe(arr => {
  console.log('buffer:', arr);
  // Uscite: buffer: [0, 1]
  // Uscite: buffer: [2, 3, 4]
});
```

## ⚠️ Note.

### 1. Le funzioni di predicato devono essere funzioni pure.

Le funzioni di predicato con effetti collaterali possono causare un comportamento inatteso quando lo stream viene risottoscritto.

```ts
// ❌ Esempio negativo: Effetti collaterali Sì
let counter = 0;
source$.pipe(
  filter(x => {
    counter++; // Effetto collaterale
    return x > 10;
  })
).subscribe();

// ✅ Buon esempio: Funzione pura
source$.pipe(
  filter(x => x > 10)
).subscribe();
```

### 2. Usare come funzione di protezione del tipo

Si può scrivere per restituire un predicato di tipo TypeScript (`x is T`) per restringere il tipo dopo aver passato filter`.

```ts
import { Observable, of, filter } from 'rxjs';

interface User {
  id: number;
  name: string;
  email?: string;
}

const users$: Observable<User> = of(
  { id: 1, name: 'Alice', email: 'alice@example.com' },
  { id: 2, name: 'Bob' }
);

// Usata come funzione di protezione del tipo
users$.pipe(
  filter((user): user is User & { email: string } => user.email !== undefined)
).subscribe(user => {
  console.log(user.email.toLowerCase()); // email non è una funzione di protezione del tipo string Viene dedotta come tipo
});
```

> [!TIP] 型ガードの効果

> Restituendo il predicato di tipo `user is User & { email: string }`, `user` dopo `filtro` rende `email` una proprietà obbligatoria. Chiamate come `user.email.toLowerCase()` possono essere scritte senza errori di tipo.

## 📚 Operatori correlati.

- [take](/it/guide/operators/filtering/take) - vengono presi solo i primi N valori.
- [first](/it/guide/operators/filtering/first) - ottiene solo il primo valore (può anche essere condizionale).
- distinct](/it/guide/operators/filtering/distinct) - esclude i valori duplicati.
- [distinctUntilChanged](/it/guide/operators/filtering/distinctUntilChanged) - esclude i valori uguali all'ultimo valore

## Riepilogo.

L'operatore `filter` è lo strumento di filtraggio più elementare di RxJS.

- Passa solo i valori che corrispondono alle condizioni.
- Può essere usato allo stesso modo di `.filter()` per gli array.
- ✅ Può essere usato anche come type guard di TypeScript.
- ⚠️ Le funzioni di predicato devono essere funzioni pure
- ⚠️ Nome simile, ma uso diverso da quello di buffer (valori singoli o array).
