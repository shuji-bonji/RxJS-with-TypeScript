---
description: "find è un operatore di filtraggio di RxJS che trova il primo valore che soddisfa una condizione e lo invia in output, completando immediatamente il flusso. È ideale per le situazioni in cui si desidera trovare un elemento specifico da un array o da un elenco, come la ricerca di utenti, il controllo dell'inventario o il rilevamento di log di errore. Se non viene trovato alcun valore, viene emesso undefined e in TypeScript il valore di ritorno è di tipo T | undefined."
---

# find - trova il primo valore che soddisfa la condizione

L'operatore find trova ed emette il **primo valore che soddisfa la condizione** e completa immediatamente il flusso. Se non viene trovato alcun valore, viene emesso `undefined`.

## 🔰 Sintassi e uso di base

```ts
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  find(n => n % 2 === 0)
).subscribe(console.log);
// Uscita.: 8(primo numero pari)
```

**Flusso delle operazioni**:.
1. controllo 1, 3, 5, 7 → condizione non soddisfatta
2. controllo 8 → condizione soddisfatta → uscita 8 e completa
3. 9, 10 non valutati

[🌐 Documentazione ufficiale di RxJS - find](https://rxjs.dev/api/operators/find)

## 🆚 Contrasto con first

Find e first sono simili, ma il loro uso è diverso.

{\AN8}CONOSCENZA_2___

| Operatore. | Specifica della condizione | Se non viene trovato alcun valore | Caso d'uso. |
|---|---|---|---|
| first() | Opzione | Errore (`EmptyError`) | Ottenere il primo valore |
| first(predicato) | Opzionale | Errore (`EmptyError`) | Ottenere in modo condizionale. |
| find(predicato) | Richiesto. | Uscita `indefinita`. | Ricerca e controllo dell'esistenza |

## 💡 Modello di utilizzo tipico

1. **Ricerca dell'utente**.

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

   // ID(la condizione è facoltativa)2Ricerca di utenti con
   users$.pipe(
     find(user => user.id === 2)
   ).subscribe(user => {
     if (user) {
       console.log(`Trovato: ${user.name}`);
     } else {
       console.log('Utente non trovato');
     }
   });
   // Uscita.: Trovato: Bob
   ```

2. **Controllo dell'inventario**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface Product {
     id: string;
     name: string;
     stock: number;
   }

   const products$ = from([
     { id: 'A1', name: 'TaccuinoPC', stock: 0 },
     { id: 'A2', name: 'Mouse', stock: 15 },
     { id: 'A3', name: 'Tastiere', stock: 8 }
   ] as Product[]);

   // Scopri cosa è esaurito
   products$.pipe(
     find(product => product.stock === 0)
   ).subscribe(product => {
     if (product) {
       console.log(`Esaurito: ${product.name}`);
     } else {
       console.log('Tutti in magazzino');
     }
   });
   // Uscita.: Esaurito: TaccuinoPC
   ```

3. **Ricerca del registro degli errori**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface LogEntry {
     timestamp: number;
     level: 'info' | 'warn' | 'error';
     message: string;
   }

   const logs$ = from([
     { timestamp: 1, level: 'info' as const, message: 'App started' },
     { timestamp: 2, level: 'info' as const, message: 'User logged in' },
     { timestamp: 3, level: 'error' as const, message: 'Connection failed' },
     { timestamp: 4, level: 'info' as const, message: 'Retry successful' }
   ] as LogEntry[]);

   // Ricerca del primo errore
   logs$.pipe(
     find(log => log.level === 'error')
   ).subscribe(log => {
     if (log) {
       console.log(`Rilevamento dell'errore: ${log.message} (Tempo: ${log.timestamp})`);
     }
   });
   // Uscita.: Rilevamento dell'errore: Connection failed (Tempo: 3)
   ```

## 🧠 Esempio pratico di codice (ricerca di prodotti)

Questo è un esempio di ricerca di prodotti corrispondenti a criteri specifici dal magazzino.

```

ts.
import { from, fromEvent } da 'rxjs';
import { find } from 'rxjs';

interfaccia Prodotto {
  id: stringa;
  nome: stringa
  prezzo: numero
  categoria: stringa;
}

const products: Product[] = [
  { id: 'P1', nome: 'Mouse wireless', prezzo: 2980, categoria: 'Periferiche PC' }
  { id: 'P2', nome: 'Tastiera meccanica', prezzo: 8980, categoria: 'Periferiche PC' }
  { id: 'P3', nome: 'Chiavetta USB 64GB', prezzo: 1480, categoria: 'Archiviazione' }
  { id: 'P4', nome: 'Monitor 27 pollici', prezzo: 29800, categoria: 'Display' }
  { id: 'P5', nome: 'Supporto per laptop', prezzo: 3980, categoria: 'Periferiche per PC' }; id: 'P5', nome: 'Supporto per laptop', prezzo: 3980, categoria: 'Periferiche per PC' }
];

// Creazione di elementi dell'interfaccia utente
const container = document.createElement('div');.
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Ricerca prodotti';
container.appendChild(title);

const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Inserisci il prezzo massimo';
input.style.marginRight = '10px';
container.appendChild(input);

const searchButton = document.createElement('button');
searchButton.textContent = 'search';
container.appendChild(searchButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
container.appendChild(result);

// Elaborazione della ricerca
// Nota: originariamente lo schema consigliato è di appiattire con una switchMap, ma,
// Nota: sebbene il modello raccomandato sia quello di appiattire con una switchMap, // qui annidiamo il subscribe per la leggibilità, // perché include la validazione dell'interfaccia utente (ritorno anticipato).
// Considerare un'implementazione piatta, utilizzando switchMap, nel codice di produzione.
fromEvent(searchButton, 'click').subscribe(() => {
  const maxPrice = parseInt(input.value);.

  if (isNaN(maxPrice)) {
    result.textContent = 'Inserisci un prezzo';
    result.style.color = 'red';
    return;
  }

  // Nest subscribe: originariamente si consigliava di appiattire con switchMap
  from(prodotti).pipe(
    find(product => product.price <= maxPrice)
  ).subscribe(prodotto => {
    if (prodotto) {
      result.innerHTML = `
        <strong>Trovato! </strong><br>
        Nome del prodotto: ${product.name}<br>
        Prezzo: ${product.price.toLocaleString()}<br>
        Categoria: ${product.category}
      `;
      result.style.color = 'green';
    } else {
      result.textContent = `¥${maxPrice.toLocaleString()} o meno prodotto non trovato `;
      result.style.color = 'orange'; } else { result.textContent = `¥${maxPrice.tolocaleStar()} o meno prodotto non trovato `; result.style.color = 'orange'; }
    }
  });
});

```

Questo codice cerca e visualizza il primo prodotto al di sotto del prezzo inserito dall'utente.

## 🎯 filter La differenza tra

`find` e `filter` sono utilizzati per scopi diversi.

```

ts.
import { from } da 'rxjs';
import { find, filter } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// filter: restituisce tutti i valori che corrispondono alla condizione
numbers$.pipe(
  filter(n => n > 5)
).subscribe({
  next: console.log,.
  complete: () => console.log('filter complete')
});
// Uscita: 7, 8, 9, 10, filtro completato

// find: produce solo il primo valore che corrisponde alla condizione
numbers$.pipe(
  find(n => n > 5)
).subscribe({
  next: console.log,.
  complete: () => console.log('find complete')
});
// uscita: 7, find complete

```

| Operatore | Numero di uscite | Tempi di completamento | Caso d'uso |
|---|---|---|---|
| `filter(predicate)` | Tutti i valori che corrispondono alla condizione | Al completamento del flusso originale | Raffinamento dei dati |
| `find(predicate)` | Solo il primo valore che corrisponde ai criteri | Immediatamente dopo la scoperta | Ricerca e controllo dell'esistenza |

## 📋 Utilizzo sicuro per i tipi

TypeScript Questo è un esempio di implementazione type-safe che utilizza i generici in

```

ts.
import { Observable, from } da 'rxjs';
import { find } from 'rxjs';

interfaccia Task {
  id: numero;
  titolo: stringa
  completato: booleano;
  priorità: 'alta' | 'media' | 'bassa'; }
}

function findTaskById(
  tasks$: Observable,.
  id: numero
): Observable | undefined> {
  return tasks$.pipe(
    find(task => task.id === id)
  );
}

function findFirstIncompleteTask(
  tasks$: Observable
): Observable | undefined> {
  return tasks$.pipe(
    find(task => !task.complete)
  );
}

// Esempio di utilizzo
const tasks$ = from([.
  { id: 1, titolo: 'Compito A', completato: true, priorità: 'alta' as const }
  { id: 2, titolo: 'Compito B', completato: false, priorità: 'media' as const }
  { id: 3, title: 'Task C', completed: false, priority: 'low' as const }
] come Task[]);.

// Ricerca per ID
findTaskById(tasks$, 2).subscribe(task => {
  if (task) {
    console.log(`trovato: ${task.title}`);
  } else {
    console.log('Compito non trovato'); }
  }
});
// Output: trovato: compito B

// Trovare i compiti non completati
findFirstIncompleteTask(tasks$).subscribe(task => {
  if (task) {
    console.log(`Prossimo compito: ${task.title} (priorità: ${task.priority})`);
  }
});
// Output: prossimo task: task B (priorità: media)

```

## 🔄 find e findIndex La differenza tra

RxJSnegli operatori `findIndex` sono disponibili anche gli operatori

```

ts
import { from } da 'rxjs';
import { find, findIndex } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// find: restituisce un valore
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log);.
// uscita: 30

// findIndex: restituisce l'indice
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);.
// Output: 2 (indice di 30)

```

| Operatore | Valore di ritorno | se il valore non viene trovato |
|---|---|---|
| `find(predicate)` | Valore stesso | `undefined` |
| `findIndex(predicate)` | Indice (valore numerico) | `-1` |

## ⚠️ Errori comuni

> [!NOTE]
> `find` se il valore non viene trovato. `undefined` viene emesso. Questo non comporta un errore. Se è richiesto un errore, utilizzare `first` da utilizzare.

### Errore.: Gestione dell'errore previsto se il valore non viene trovato.

```

ts.
import { from } da 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7]);

// ❌ Cattivo esempio: la gestione degli errori è prevista ma non viene chiamata
numbers$.pipe(
  find(n => n > 10)
).subscribe({
  next: console.log,.
  error: err => console.log('Error:', err) // non chiamato
});
// uscita: indefinita

```

### Positivo: undefined Controlla o first utilizzare il valore

```

ts.
importare { from } da 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7]);

// ✅ Buon esempio 1: controllo degli indefiniti
numbers$.pipe(
  find(n => n > 10)
).subscribe(risultato => {
  if (result ! == undefined) {
    console.log('Trovato:', risultato);
  } else {
    console.log('Non trovato:'); }
  }
});
// Output: non trovato

// ✅ Buon esempio 2: usare il first se si ha bisogno di un errore
numbers$.pipe(
  first(n => n > 10, 0) // specifica il valore predefinito
).subscribe({
  next: console.log,.
  error: err => console.log('Error:', err.message)
});
// Uscita: 0
```

## 🎓 Sommario

### Quando si dovrebbe usare find.
- ✅ Quando si vuole trovare il primo valore che soddisfa una condizione
- ✅ Quando si vuole verificare l'esistenza di un valore
- ✅ Quando si vuole trattare un valore come "non definito" se non viene trovato.
- ✅ Quando si desidera trovare un elemento specifico in una matrice o in un elenco

### Quando si dovrebbe usare first
- ✅ Se si vuole ottenere il primo valore
- ✅ Se si vuole emettere un errore se il valore non è trovato

### Quando si dovrebbe usare il filtro?
- ✅ Se si ha bisogno di tutti i valori che corrispondono a una condizione
- ✅ Se si vogliono filtrare i dati

### Note.
- ⚠️ `find` restituisce `undefined` se non viene trovato (non è un errore).
- ⚠️ Completa immediatamente con il primo valore che soddisfa la condizione
- ⚠️ TypeScript restituisce un valore di tipo `T | undefined`.

## 🚀 Passo successivo.

- **[first](. /first)** - imparare a ottenere il primo valore.
- **[filter](. /filter)** - imparare a filtrare in base alle condizioni.
- **[findIndex](https://rxjs.dev/api/operators/findIndex)** - per imparare a ottenere l'indice del primo valore che soddisfa una condizione (documentazione ufficiale).
- **[filtering-operator-practical-use-cases](. /practical-use-cases)** - per imparare casi d'uso reali.
