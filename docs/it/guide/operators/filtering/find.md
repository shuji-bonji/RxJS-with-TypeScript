---
description: find è un operatore di filtraggio RxJS che trova ed emette il primo valore che soddisfa una condizione e completa immediatamente lo stream. È ideale per scenari dove vuoi cercare un elemento specifico da un array o lista, come ricerca utenti, controllo inventario e rilevamento log errori. Se nessun valore viene trovato, emette undefined, e in TypeScript il valore di ritorno è di tipo T | undefined.
titleTemplate: ':title | RxJS'
---

# find - Trova il Primo Valore che Soddisfa una Condizione

L'operatore `find` trova ed emette il **primo valore che soddisfa una condizione** e completa immediatamente lo stream. Se nessun valore viene trovato, emette `undefined`.


## 🔰 Sintassi e Utilizzo Base

```ts
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  find(n => n % 2 === 0)
).subscribe(console.log);
// Output: 8 (primo numero pari)
```

**Flusso di operazione**:
1. Controlla 1, 3, 5, 7 → Non soddisfano la condizione
2. Controlla 8 → Soddisfa la condizione → Emetti 8 e completa
3. 9, 10 non vengono valutati

[🌐 Documentazione Ufficiale RxJS - `find`](https://rxjs.dev/api/operators/find)


## 🆚 Confronto con first

`find` e `first` sono simili ma usati diversamente.

```ts
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// first: Primo valore che soddisfa la condizione (condizione opzionale)
numbers$.pipe(
  first(n => n > 5)
).subscribe(console.log);
// Output: 7

// find: Primo valore che soddisfa la condizione (condizione obbligatoria)
numbers$.pipe(
  find(n => n > 5)
).subscribe(console.log);
// Output: 7
```

| Operatore | Specifica Condizione | Quando Valore Non Trovato | Caso d'Uso |
|---|---|---|---|
| `first()` | Opzionale | Errore (`EmptyError`) | Ottieni primo valore |
| `first(predicate)` | Opzionale | Errore (`EmptyError`) | Ottieni condizionale |
| `find(predicate)` | Obbligatoria | Emetti `undefined` | Ricerca/verifica esistenza |


## 💡 Pattern di Utilizzo Tipici

1. **Ricerca Utente**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface User {
     id: number;
     name: string;
     email: string;
   }

   const users$ = from([
     { id: 1, name: 'Alice', email: 'alice@example.com' },
     { id: 2, name: 'Bob', email: 'bob@example.com' },
     { id: 3, name: 'Charlie', email: 'charlie@example.com' }
   ] as User[]);

   // Cerca utente con ID 2
   users$.pipe(
     find(user => user.id === 2)
   ).subscribe(user => {
     if (user) {
       console.log(`Trovato: ${user.name}`);
     } else {
       console.log('Utente non trovato');
     }
   });
   // Output: Trovato: Bob
   ```

2. **Controllo Inventario**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface Product {
     id: string;
     name: string;
     stock: number;
   }

   const products$ = from([
     { id: 'A1', name: 'Laptop', stock: 0 },
     { id: 'A2', name: 'Mouse', stock: 15 },
     { id: 'A3', name: 'Tastiera', stock: 8 }
   ] as Product[]);

   // Trova prodotto esaurito
   products$.pipe(
     find(product => product.stock === 0)
   ).subscribe(product => {
     if (product) {
       console.log(`Esaurito: ${product.name}`);
     } else {
       console.log('Tutto disponibile');
     }
   });
   // Output: Esaurito: Laptop
   ```


## 🎯 Differenza da filter

`find` e `filter` sono usati per scopi diversi.

```ts
import { from } from 'rxjs';
import { find, filter } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// filter: Emetti tutti i valori che corrispondono alla condizione
numbers$.pipe(
  filter(n => n > 5)
).subscribe({
  next: console.log,
  complete: () => console.log('filter completato')
});
// Output: 7, 8, 9, 10, filter completato

// find: Emetti solo il primo valore che corrisponde alla condizione
numbers$.pipe(
  find(n => n > 5)
).subscribe({
  next: console.log,
  complete: () => console.log('find completato')
});
// Output: 7, find completato
```

| Operatore | Conteggio Output | Timing Completamento | Caso d'Uso |
|---|---|---|---|
| `filter(predicate)` | Tutti i valori che corrispondono | Quando lo stream originale completa | Filtraggio dati |
| `find(predicate)` | Solo primo valore che corrisponde | Immediatamente quando trovato | Ricerca/verifica esistenza |


## ⚠️ Errori Comuni

> [!NOTE]
> `find` emette `undefined` quando il valore non viene trovato. Non genera errore. Usa `first` se hai bisogno di un errore.

### Sbagliato: Aspettarsi Gestione Errori Quando il Valore Non Viene Trovato

```ts
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7]);

// ❌ Esempio sbagliato: Aspettandosi gestione errori ma non chiamato
numbers$.pipe(
  find(n => n > 10)
).subscribe({
  next: console.log,
  error: err => console.log('Errore:', err) // Non chiamato
});
// Output: undefined
```

### Corretto: Controlla undefined o Usa first

```ts
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7]);

// ✅ Esempio corretto 1: Controlla undefined
numbers$.pipe(
  find(n => n > 10)
).subscribe(result => {
  if (result !== undefined) {
    console.log('Trovato:', result);
  } else {
    console.log('Non trovato');
  }
});
// Output: Non trovato

// ✅ Esempio corretto 2: Usa first se serve errore
numbers$.pipe(
  first(n => n > 10, 0) // Specifica valore predefinito
).subscribe({
  next: console.log,
  error: err => console.log('Errore:', err.message)
});
// Output: 0
```


## 🎓 Riepilogo

### Quando Usare find
- ✅ Quando vuoi cercare il primo valore che soddisfa una condizione
- ✅ Quando vuoi verificare l'esistenza di un valore
- ✅ Quando vuoi gestire il caso non trovato con `undefined`
- ✅ Quando vuoi cercare un elemento specifico da un array o lista

### Quando Usare first
- ✅ Quando vuoi ottenere il primo valore
- ✅ Quando vuoi emettere un errore se il valore non viene trovato

### Quando Usare filter
- ✅ Quando hai bisogno di tutti i valori che corrispondono alla condizione
- ✅ Quando lo scopo è il filtraggio dati

### Note
- ⚠️ `find` emette `undefined` quando non trovato (non un errore)
- ⚠️ Completa immediatamente con il primo valore che soddisfa la condizione
- ⚠️ In TypeScript, il valore di ritorno è di tipo `T | undefined`


## 🚀 Prossimi Passi

- **[first](/it/guide/operators/filtering/first)** - Impara come ottenere il primo valore
- **[filter](/it/guide/operators/filtering/filter)** - Impara come filtrare in base a condizioni
- **[findIndex](https://rxjs.dev/api/operators/findIndex)** - Impara come ottenere l'indice del primo valore che soddisfa la condizione (documentazione ufficiale)
- **[Esempi Pratici Operatori di Filtraggio](/it/guide/operators/filtering/practical-use-cases)** - Impara casi d'uso reali
