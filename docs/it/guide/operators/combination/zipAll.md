---
description: zipAll è un operatore che prende un Higher-order Observable (Observable di Observable) e accoppia i valori ordinati corrispondenti di ogni Observable interno e li emette come array.
titleTemplate: ':title'
---

# zipAll - Accoppia i Valori Corrispondenti di Ogni Observable Interno

L'operatore `zipAll` prende un **Higher-order Observable** (Observable di Observable),
**accoppia i valori ordinati corrispondenti di ogni Observable interno** e li emette come array.

## 🔰 Sintassi e Utilizzo Base

```ts
import { interval, of } from 'rxjs';
import { zipAll, take } from 'rxjs';

// Higher-order Observable con tre Observable interni
const higherOrder$ = of(
  interval(1000).pipe(take(3)), // 0, 1, 2
  interval(500).pipe(take(4)),  // 0, 1, 2, 3
  interval(2000).pipe(take(2))  // 0, 1
);

// Accoppia i valori ordinati corrispondenti di ogni Observable interno
higherOrder$
  .pipe(zipAll())
  .subscribe(values => console.log(values));

// Output:
// [0, 0, 0] ← Tutti i 1° valori
// [1, 1, 1] ← Tutti i 2° valori
// (Completa qui: il 3° Observable emette solo 2 valori)
```

- Raccoglie gli Observable interni quando il Higher-order Observable **completa**
- **Accoppia i valori dello stesso indice** di ogni Observable interno
- **Quando l'Observable interno più corto completa**, tutto completa

[🌐 Documentazione Ufficiale RxJS - `zipAll`](https://rxjs.dev/api/index/function/zipAll)

## 💡 Pattern di Utilizzo Tipici

- **Abbinare più risposte API in sequenza**
- **Confrontare valori dello stesso timing di più stream**
- **Combinare risultati di elaborazione parallela in sequenza**

## 🧠 Esempio di Codice Pratico

Esempio di accoppiamento dei valori corrispondenti di più contatori

```ts
import { interval, of } from 'rxjs';
import { zipAll, take, map } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

// Crea tre contatori con velocità diverse
const counters$ = of(
  interval(1000).pipe(take(4), map(n => `Lento: ${n}`)),
  interval(500).pipe(take(5), map(n => `Normale: ${n}`)),
  interval(300).pipe(take(6), map(n => `Veloce: ${n}`))
);

// Accoppia i valori ordinati corrispondenti di ogni contatore
counters$
  .pipe(zipAll())
  .subscribe(values => {
    const item = document.createElement('div');
    item.textContent = `[${values.join(', ')}]`;
    output.appendChild(item);
  });

// Output:
// [Lento: 0, Normale: 0, Veloce: 0]
// [Lento: 1, Normale: 1, Veloce: 1]
// [Lento: 2, Normale: 2, Veloce: 2]
// [Lento: 3, Normale: 3, Veloce: 3]
// (Completa qui: il contatore "Lento" emette solo 4 valori)
```

## 🔄 Funzione di Creazione Correlata

Mentre `zipAll` è principalmente usato per appiattire Higher-order Observable,
usa la **Funzione di Creazione** `zip` per il normale accoppiamento di più Observable.

```ts
import { zip, interval } from 'rxjs';
import { take } from 'rxjs';

// Versione Funzione di Creazione (utilizzo più comune)
const zipped$ = zip(
  interval(1000).pipe(take(3)),
  interval(500).pipe(take(4)),
  interval(2000).pipe(take(2))
);

zipped$.subscribe(console.log);
```

Vedi [Capitolo 3: Funzioni di Creazione - zip](/it/guide/creation-functions/combination/zip).

## 🔄 Operatori Correlati

| Operatore | Descrizione |
|---|---|
| [combineLatestAll](/it/guide/operators/combination/combineLatestAll) | Combina gli ultimi valori di tutti gli Observable interni |
| [mergeAll](/it/guide/operators/combination/mergeAll) | Sottoscrive tutti gli Observable interni in parallelo |
| [concatAll](/it/guide/operators/combination/concatAll) | Sottoscrive gli Observable interni in ordine |
| [switchAll](/it/guide/operators/combination/switchAll) | Passa al nuovo Observable interno |

## 🔄 zipAll vs combineLatestAll

| Operatore | Metodo di Combinazione | Timing di Completamento |
|---|---|---|
| `zipAll` | Accoppia valori allo **stesso indice** | Quando l'Observable interno **più corto** completa |
| `combineLatestAll` | Combina **ultimi valori** | Quando **tutti** gli Observable interni completano |

```ts
// zipAll: [0°, 0°, 0°], [1°, 1°, 1°], ...
// combineLatestAll: [ultimo, ultimo, ultimo], [ultimo, ultimo, ultimo], ...
```

## ⚠️ Note Importanti

### Il Higher-order Observable Deve Completare

`zipAll` attende di raccogliere gli Observable interni fino a quando il Higher-order Observable (Observable esterno) **completa**.

#### ❌ Nessun output perché il Higher-order Observable non completa
```ts
interval(1000).pipe(
  map(() => of(1, 2, 3)),
  zipAll()
).subscribe(console.log); // Nessun output
```

#### ✅ Completa con take
```ts
interval(1000).pipe(
  take(3), // Completa dopo 3
  map(() => of(1, 2, 3)),
  zipAll()
).subscribe(console.log);
```

### Completa con l'Observable Interno Più Corto

Quando l'**Observable interno più corto completa**, tutto completa.

```ts
import { of, zipAll } from "rxjs";

of(
  of(1, 2, 3, 4, 5), // 5 valori
  of(1, 2)           // 2 valori ← Più corto
).pipe(
  zipAll()
).subscribe(console.log);

// Output: [1, 1], [2, 2]
// (Completa a 2. 3, 4, 5 non vengono usati)
```

### Backpressure (Utilizzo Memoria)

Quando gli Observable interni emettono a velocità diverse, **i valori dagli Observable interni più veloci si accumulano in memoria**.

```ts
import { interval, of, take, zipAll } from "rxjs";

// I valori del contatore veloce (100ms) si accumulano in memoria mentre aspettano il contatore lento (10000ms)
of(
  interval(10000).pipe(take(3)), // Lento
  interval(100).pipe(take(100))  // Veloce
).pipe(
  zipAll()
).subscribe(console.log);
```

Se la differenza di velocità è grande, presta attenzione all'utilizzo della memoria.
