---
description: L'operatore findIndex è un operatore di filtraggio RxJS che restituisce l'indice del primo valore che soddisfa la condizione. Se non trovato, restituisce -1.
titleTemplate: ':title | RxJS'
---

# findIndex - Ottieni l'Indice del Primo Valore che Corrisponde alla Condizione

L'operatore `findIndex` restituisce **l'indice del primo valore che soddisfa la condizione** e completa immediatamente lo stream. Se il valore non viene trovato, restituisce `-1`.

## 🔰 Sintassi e Utilizzo Base

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  findIndex(n => n % 2 === 0)
).subscribe(console.log);
// Output: 4 (indice del primo numero pari 8)
```

**Flusso di operazione**:
1. 1 (indice 0) → Dispari, salta
2. 3 (indice 1) → Dispari, salta
3. 5 (indice 2) → Dispari, salta
4. 7 (indice 3) → Dispari, salta
5. 8 (indice 4) → Pari, emetti indice 4 e completa

[🌐 Documentazione Ufficiale RxJS - `findIndex`](https://rxjs.dev/api/operators/findIndex)

## 💡 Pattern di Utilizzo Tipici

- **Individua posizione nell'array**: Ottieni posizione dell'elemento che corrisponde a una condizione specifica
- **Verifica ordine**: Determina a quale posizione appare un elemento che corrisponde a una condizione
- **Ordinamento dati**: Elaborazione usando informazioni sull'indice
- **Verifica esistenza**: Controlla esistenza verificando se è -1 o meno

## 🆚 Confronto con Operatori Simili

### findIndex vs find vs elementAt

```ts
import { from } from 'rxjs';
import { findIndex, find, elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// findIndex: Restituisci indice del primo valore che corrisponde alla condizione
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);
// Output: 2 (indice di 30)

// find: Restituisci primo valore che corrisponde alla condizione
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log);
// Output: 30

// elementAt: Restituisci valore all'indice specificato
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Output: 30
```

| Operatore | Argomento | Valore Restituito | Quando Non Trovato |
|:---|:---|:---|:---|
| `findIndex(predicate)` | Funzione condizione | Indice (number) | `-1` |
| `find(predicate)` | Funzione condizione | Valore stesso | `undefined` |
| `elementAt(index)` | Indice | Valore stesso | Errore (nessun valore predefinito) |

## ⚠️ Note Importanti

### 1. Restituisce -1 se Non Trovato

Se nessun valore soddisfa la condizione, restituisce `-1` invece di un errore.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 9]);

numbers$.pipe(
  findIndex(n => n > 10)
).subscribe(index => {
  if (index === -1) {
    console.log('Nessun valore corrispondente alla condizione trovato');
  } else {
    console.log(`Indice: ${index}`);
  }
});
// Output: Nessun valore corrispondente alla condizione trovato
```

### 2. Completa Quando Trova la Prima Corrispondenza

Lo stream completa immediatamente quando trova il primo valore che corrisponde alla condizione.

```ts
import { interval } from 'rxjs';
import { findIndex, tap } from 'rxjs';

interval(1000).pipe(
  tap(val => console.log(`Valore: ${val}`)),
  findIndex(n => n >= 3)
).subscribe(index => {
  console.log(`Indice: ${index}`);
});
// Output:
// Valore: 0
// Valore: 1
// Valore: 2
// Valore: 3
// Indice: 3
```

### 3. Type Safety in TypeScript

`findIndex` restituisce sempre un tipo `number`.

```ts
import { Observable, from } from 'rxjs';
import { findIndex } from 'rxjs';

interface User {
  id: number;
  name: string;
  isActive: boolean;
}

function findFirstInactiveUserIndex(
  users$: Observable<User>
): Observable<number> {
  return users$.pipe(
    findIndex(user => !user.isActive)
  );
}

const users$ = from([
  { id: 1, name: 'Alice', isActive: true },
  { id: 2, name: 'Bob', isActive: false },
  { id: 3, name: 'Charlie', isActive: true }
]);

findFirstInactiveUserIndex(users$).subscribe(index => {
  // index è di tipo number
  if (index !== -1) {
    console.log(`Il primo utente inattivo è all'indice ${index}`);
  }
});
// Output: Il primo utente inattivo è all'indice 1
```

### 4. Gli Indici Iniziano da 0

Come gli array, gli indici iniziano da 0.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const items$ = from(['A', 'B', 'C', 'D']);

items$.pipe(
  findIndex(item => item === 'A')
).subscribe(console.log);
// Output: 0 (primo elemento)
```

## 📚 Operatori Correlati

- **[find](/it/guide/operators/filtering/find)** - Ottieni primo valore che corrisponde alla condizione
- **[elementAt](/it/guide/operators/filtering/elementAt)** - Ottieni valore all'indice specificato
- **[first](/it/guide/operators/filtering/first)** - Ottieni primo valore
- **[filter](/it/guide/operators/filtering/filter)** - Ottieni tutti i valori che corrispondono alla condizione

## Riepilogo

L'operatore `findIndex` restituisce l'indice del primo valore che corrisponde alla condizione.

- ✅ Comportamento simile a `Array.findIndex()` di JavaScript
- ✅ Ideale quando serve informazione sull'indice
- ✅ Restituisce `-1` se non trovato (non un errore)
- ✅ Completa immediatamente quando trovato
- ⚠️ Il valore restituito è sempre di tipo `number` (-1 o intero ≥ 0)
- ⚠️ Usa `find` se hai bisogno del valore stesso
