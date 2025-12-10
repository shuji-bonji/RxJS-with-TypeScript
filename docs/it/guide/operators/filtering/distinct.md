---
description: L'operatore distinct rimuove tutti i valori duplicati ed emette solo valori unici che non sono mai stati emessi. È necessario prestare attenzione con stream infiniti, poiché utilizza internamente Set per memorizzare i valori precedentemente emessi.
---

# distinct - Rimuovi Tutti i Valori Duplicati

L'operatore `distinct` monitora tutti i valori emessi dall'Observable ed emette **solo valori che non sono mai stati emessi prima**. Internamente, utilizza Set per ricordare i valori precedentemente emessi.


## 🔰 Sintassi e Utilizzo Base

```ts
import { of } from 'rxjs';
import { distinct } from 'rxjs';

const numbers$ = of(1, 2, 1, 3, 2, 4, 1, 5);

numbers$.pipe(
  distinct()
).subscribe(console.log);
// Output: 1, 2, 3, 4, 5
```

- Rimuove i duplicati in tutto lo stream
- Una volta emesso un valore, viene ignorato indipendentemente da quante volte appare successivamente
- `distinctUntilChanged` rimuove solo i duplicati **consecutivi**, mentre `distinct` rimuove **tutti** i duplicati

[🌐 Documentazione Ufficiale RxJS - `distinct`](https://rxjs.dev/api/operators/distinct)


## 🆚 Differenza da distinctUntilChanged

```ts
import { of } from 'rxjs';
import { distinct, distinctUntilChanged } from 'rxjs';

const values$ = of(1, 2, 1, 2, 3, 1, 2, 3);

// distinctUntilChanged: Rimuovi solo duplicati consecutivi
values$.pipe(
  distinctUntilChanged()
).subscribe(console.log);
// Output: 1, 2, 1, 2, 3, 1, 2, 3

// distinct: Rimuovi tutti i duplicati
values$.pipe(
  distinct()
).subscribe(console.log);
// Output: 1, 2, 3
```

| Operatore | Target Rimozione | Caso d'Uso |
|---|---|---|
| `distinctUntilChanged` | Solo duplicati consecutivi | Campi input, dati sensori |
| `distinct` | Tutti i duplicati | Lista valori unici, lista ID |


## 🎯 Personalizzazione Confronto con keySelector

Usa la funzione `keySelector` per determinare i duplicati per una proprietà specifica di un oggetto.

```ts
import { of } from 'rxjs';
import { distinct } from 'rxjs';

interface User {
  id: number;
  name: string;
}

const users$ = of(
  { id: 1, name: 'Alice' } as User,
  { id: 2, name: 'Bob' } as User,
  { id: 1, name: 'Alice (aggiornato)' } as User, // Stesso ID
  { id: 3, name: 'Charlie' } as User
);

users$.pipe(
  distinct(user => user.id) // Determina duplicati per ID
).subscribe(console.log);
// Output:
// { id: 1, name: 'Alice' }
// { id: 2, name: 'Bob' }
// { id: 3, name: 'Charlie' }
```


## 💡 Pattern di Utilizzo Tipici

1. **Ottieni Lista di ID Unici**
   ```ts
   import { from } from 'rxjs';
   import { distinct, map } from 'rxjs';

   interface Order {
     orderId: string;
     userId: number;
     amount: number;
   }

   const orders$ = from([
     { orderId: 'A1', userId: 1, amount: 100 },
     { orderId: 'A2', userId: 2, amount: 200 },
     { orderId: 'A3', userId: 1, amount: 150 },
     { orderId: 'A4', userId: 3, amount: 300 }
   ] as Order[]);

   // Ottieni solo ID utente unici
   orders$.pipe(
     map(order => order.userId),
     distinct()
   ).subscribe(userId => {
     console.log(`ID Utente: ${userId}`);
   });
   // Output: 1, 2, 3
   ```

2. **Estrai Tipi di Evento Unici dal Log Eventi**
   ```ts
   import { fromEvent, merge } from 'rxjs';
   import { map, distinct, take } from 'rxjs';

   // Crea elementi UI dinamicamente
   const container = document.createElement('div');
   document.body.appendChild(container);

   const button1 = document.createElement('button');
   button1.textContent = 'Pulsante 1';
   container.appendChild(button1);

   const button2 = document.createElement('button');
   button2.textContent = 'Pulsante 2';
   container.appendChild(button2);

   const input = document.createElement('input');
   input.placeholder = 'Inserisci';
   container.appendChild(input);

   const log = document.createElement('div');
   log.style.marginTop = '10px';
   container.appendChild(log);

   // Unisci più stream di eventi per estrarre tipi di evento unici
   const events$ = merge(
     fromEvent(button1, 'click').pipe(map(() => 'button1-click')),
     fromEvent(button2, 'click').pipe(map(() => 'button2-click')),
     fromEvent(input, 'input').pipe(map(() => 'input-change'))
   );

   events$.pipe(
     distinct(),
     take(3) // Completa quando tutti i 3 tipi di eventi sono presenti
   ).subscribe({
     next: (eventType) => {
       log.textContent += `Evento unico: ${eventType}\n`;
       console.log(`Evento unico: ${eventType}`);
     },
     complete: () => {
       log.textContent += 'Tutti i tipi di evento rilevati';
     }
   });
   ```


## 🧠 Esempio di Codice Pratico (Input Tag)

Ecco un esempio di UI che rimuove automaticamente i duplicati dai tag inseriti dall'utente.

```ts
import { fromEvent, Subject } from 'rxjs';
import { map, distinct, scan } from 'rxjs';

// Crea elementi UI
const container = document.createElement('div');
document.body.appendChild(container);

const tagInput = document.createElement('input');
tagInput.type = 'text';
tagInput.placeholder = 'Inserisci tag e premi Invio';
container.appendChild(tagInput);

const tagList = document.createElement('ul');
tagList.style.marginTop = '10px';
container.appendChild(tagList);

// Stream aggiunta tag
const tagSubject$ = new Subject<string>();

tagSubject$.pipe(
  map(tag => tag.trim().toLowerCase()),
  distinct() // Rimuovi tag duplicati
).subscribe(tag => {
  const li = document.createElement('li');
  li.textContent = tag;
  tagList.appendChild(li);
});

// Aggiungi un tag con il tasto Invio
fromEvent<KeyboardEvent>(tagInput, 'keydown').subscribe(event => {
  if (event.key === 'Enter') {
    const value = tagInput.value.trim();
    if (value) {
      tagSubject$.next(value);
      tagInput.value = '';
    }
  }
});
```

Questo codice assicura che lo stesso tag venga aggiunto alla lista solo una volta, anche se viene inserito più volte.


## ⚠️ Nota sull'Utilizzo della Memoria

> [!WARNING]
> L'operatore `distinct` utilizza **Set** internamente per memorizzare tutti i valori precedentemente emessi. Usarlo con uno stream infinito può causare memory leak.

### Problema: Memory Leak negli Stream Infiniti

```ts
import { interval } from 'rxjs';
import { distinct, map } from 'rxjs';

// ❌ Esempio sbagliato: Usare distinct con stream infiniti
interval(100).pipe(
  map(n => n % 10), // Ciclo 0-9
  distinct() // Emette solo i primi 10, poi li mantiene in memoria
).subscribe(console.log);
// Output: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
// Niente viene emesso dopo, ma il Set continua ad essere memorizzato
```

### Soluzione: Svuota il Set con il Parametro flushes

```ts
import { interval, timer } from 'rxjs';
import { distinct, map } from 'rxjs';

// ✅ Esempio corretto: Svuota periodicamente il Set
interval(100).pipe(
  map(n => n % 5),
  distinct(
    value => value,
    timer(1000) // Svuota il Set ogni 1 secondo
  )
).subscribe(console.log);
// Ogni 1 secondo, 0, 1, 2, 3, 4 vengono ri-emessi
```

### Best Practice

1. **Usa con stream finiti**: Risposte HTTP, conversione da array, ecc.
2. **Usa flushes**: Svuota periodicamente per stream infiniti
3. **Considera distinctUntilChanged**: Usalo per rimuovere solo duplicati consecutivi


## 📋 Utilizzo Type-Safe

Ecco un esempio di implementazione type-safe utilizzando i generics di TypeScript.

```ts
import { Observable } from 'rxjs';
import { distinct, map } from 'rxjs';

interface Product {
  id: number;
  name: string;
  categoryId: number;
}

function getUniqueCategories(
  products$: Observable<Product>
): Observable<number> {
  return products$.pipe(
    distinct(product => product.categoryId)
  ).pipe(
    map(product => product.categoryId)
  );
}

// Esempio di utilizzo
import { of } from 'rxjs';

const products$ = of(
  { id: 1, name: 'Laptop', categoryId: 10 } as Product,
  { id: 2, name: 'Mouse', categoryId: 10 } as Product,
  { id: 3, name: 'Libro', categoryId: 20 } as Product
);

getUniqueCategories(products$).subscribe(categoryId => {
  console.log(`ID Categoria: ${categoryId}`);
});
// Output: 10, 20
```


## 🎓 Riepilogo

### Quando Usare distinct
- ✅ Quando hai bisogno di una lista di valori unici
- ✅ Quando vuoi rimuovere duplicati in uno stream finito
- ✅ Creazione di liste di ID o categorie

### Quando Usare distinctUntilChanged
- ✅ Quando vuoi rimuovere solo duplicati consecutivi
- ✅ Rilevamento cambiamenti campi input
- ✅ Quando vuoi risparmiare memoria con stream infiniti

### Note
- ⚠️ Usa il parametro `flushes` per stream infiniti per prevenire memory leak
- ⚠️ Sii consapevole dell'utilizzo della memoria quando vengono emessi grandi numeri di valori unici
- ⚠️ Se le prestazioni sono critiche, monitora la dimensione del Set


## 🚀 Prossimi Passi

- **[distinctUntilChanged](/it/guide/operators/filtering/distinctUntilChanged)** - Impara come rimuovere solo duplicati consecutivi
- **[distinctUntilKeyChanged](/it/guide/operators/filtering/distinctUntilKeyChanged)** - Impara come confrontare oggetti per chiave
- **[filter](/it/guide/operators/filtering/filter)** - Impara come filtrare in base a condizioni
- **[Esempi Pratici Operatori di Filtraggio](/it/guide/operators/filtering/practical-use-cases)** - Impara casi d'uso reali
