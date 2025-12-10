---
description: L'operatore skip salta il primo numero specificato di valori dallo stream Observable ed emette solo i valori successivi. È utile quando vuoi ignorare dati iniziali o saltare un periodo di riscaldamento.
---

# skip - Salta i Primi N Valori

L'operatore `skip` salta il **primo numero specificato** di valori dallo stream ed emette solo i valori successivi.


## 🔰 Sintassi e Utilizzo Base

```ts
import { interval } from 'rxjs';
import { skip } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  skip(3)
).subscribe(console.log);
// Output: 3, 4, 5, 6, 7, ...
```

- Salta i primi 3 valori (0, 1, 2)
- Il quarto e i valori successivi (3, 4, 5, ...) vengono tutti emessi
- Lo stream completa al momento del completamento originale

[🌐 Documentazione Ufficiale RxJS - `skip`](https://rxjs.dev/api/operators/skip)


## 🆚 Confronto con take

`skip` e `take` hanno comportamento opposto.

```ts
import { range } from 'rxjs';
import { skip, take } from 'rxjs';

const numbers$ = range(0, 10); // Da 0 a 9

// take: Ottieni i primi N valori
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Output: 0, 1, 2

// skip: Salta i primi N valori
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// Output: 3, 4, 5, 6, 7, 8, 9

// Combinazione: Salta i primi 3 e prendi i prossimi 3
numbers$.pipe(
  skip(3),
  take(3)
).subscribe(console.log);
// Output: 3, 4, 5
```

| Operatore | Comportamento | Timing Completamento |
|---|---|---|
| `take(n)` | Ottieni i primi n valori | Completa automaticamente dopo n valori |
| `skip(n)` | Salta i primi n valori | Quando lo stream originale completa |


## 💡 Pattern di Utilizzo Tipici

1. **Saltare Valori Iniziali**
   ```ts
   import { BehaviorSubject } from 'rxjs';
   import { skip } from 'rxjs';

   const state$ = new BehaviorSubject<number>(0);

   // Salta valore iniziale e monitora solo i cambiamenti
   state$.pipe(
     skip(1)
   ).subscribe(value => {
     console.log(`Stato cambiato: ${value}`);
   });

   state$.next(1); // Output: Stato cambiato: 1
   state$.next(2); // Output: Stato cambiato: 2
   ```

2. **Saltare Periodo di Riscaldamento**
   ```ts
   import { interval } from 'rxjs';
   import { skip, map } from 'rxjs';

   // Simula dati sensore
   const sensorData$ = interval(100).pipe(
     map(() => Math.random() * 100)
   );

   // Salta i primi 10 valori (1 secondo) come periodo di calibrazione
   sensorData$.pipe(
     skip(10)
   ).subscribe(data => {
     console.log(`Valore sensore: ${data.toFixed(2)}`);
   });
   ```

3. **Paginazione**
   ```ts
   import { from } from 'rxjs';
   import { skip, take } from 'rxjs';

   interface Item {
     id: number;
     name: string;
   }

   const allItems$ = from([
     { id: 1, name: 'Elemento 1' },
     { id: 2, name: 'Elemento 2' },
     { id: 3, name: 'Elemento 3' },
     { id: 4, name: 'Elemento 4' },
     { id: 5, name: 'Elemento 5' },
     { id: 6, name: 'Elemento 6' },
   ] as Item[]);

   const pageSize = 2;
   const pageNumber = 2; // Indice 0

   // Ottieni gli elementi della pagina 2 (elementi 5 e 6)
   allItems$.pipe(
     skip(pageNumber * pageSize),
     take(pageSize)
   ).subscribe(item => {
     console.log(item);
   });
   // Output: { id: 5, name: 'Elemento 5' }, { id: 6, name: 'Elemento 6' }
   ```


## 🧠 Esempio di Codice Pratico (Contatore)

Questo esempio salta i primi 3 click e conta solo dal 4° click in poi.

```ts
import { fromEvent } from 'rxjs';
import { skip, scan } from 'rxjs';

// Crea elementi UI
const container = document.createElement('div');
document.body.appendChild(container);

const button = document.createElement('button');
button.textContent = 'Click';
container.appendChild(button);

const counter = document.createElement('div');
counter.style.marginTop = '10px';
counter.textContent = 'Conteggio: 0';
container.appendChild(counter);

const message = document.createElement('div');
message.style.marginTop = '5px';
message.style.color = 'gray';
message.textContent = 'I primi 3 click saranno saltati';
container.appendChild(message);

// Evento click
fromEvent(button, 'click').pipe(
  skip(3), // Salta primi 3 click
  scan((count) => count + 1, 0)
).subscribe(count => {
  counter.textContent = `Conteggio: ${count}`;
  if (count === 1) {
    message.textContent = 'Il conteggio inizia dopo il 4° click!';
    message.style.color = 'green';
  }
});
```

Questo codice ignora i primi 3 click e inizia a contare come "1" dal 4° click.


## 🎯 Differenza tra skip e skipWhile

```ts
import { of } from 'rxjs';
import { skip, skipWhile } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5, 6);

// skip: Specifica primi N valori per numero
numbers$.pipe(
  skip(3)
).subscribe(console.log);
// Output: 4, 5, 6

// skipWhile: Salta mentre la condizione è soddisfatta
numbers$.pipe(
  skipWhile(n => n < 4)
).subscribe(console.log);
// Output: 4, 5, 6
```

| Operatore | Condizione Skip | Caso d'Uso |
|---|---|---|
| `skip(n)` | Salta i primi n valori per numero | Skip numero fisso |
| `skipWhile(predicate)` | Salta mentre la condizione è soddisfatta | Skip basato su condizione |
| `skipUntil(notifier$)` | Salta fino a quando un altro Observable si attiva | Skip basato sul tempo |


## 📋 Utilizzo Type-Safe

Ecco un esempio di implementazione type-safe utilizzando i generics di TypeScript.

```ts
import { Observable, from } from 'rxjs';
import { skip, take } from 'rxjs';

interface User {
  id: number;
  name: string;
  role: 'admin' | 'user';
}

function getPaginatedUsers(
  users$: Observable<User>,
  page: number,
  pageSize: number
): Observable<User> {
  return users$.pipe(
    skip(page * pageSize),
    take(pageSize)
  );
}

// Esempio di utilizzo
const users$ = from([
  { id: 1, name: 'Alice', role: 'admin' as const },
  { id: 2, name: 'Bob', role: 'user' as const },
  { id: 3, name: 'Charlie', role: 'user' as const },
  { id: 4, name: 'Dave', role: 'admin' as const },
  { id: 5, name: 'Eve', role: 'user' as const },
] as User[]);

// Ottieni pagina 1 (seconda pagina, indice 0)
getPaginatedUsers(users$, 1, 2).subscribe(user => {
  console.log(`${user.name} (${user.role})`);
});
// Output: Charlie (user), Dave (admin)
```


## ⚠️ Errori Comuni

> [!NOTE]
> `skip` salta solo i primi N valori e non completa lo stream. Negli stream infiniti, combina con `take` per impostare la condizione di terminazione.

### Sbagliato: Usare skip Solo con Stream Infiniti

```ts
import { interval } from 'rxjs';
import { skip } from 'rxjs';

// ❌ Esempio sbagliato: Lo stream infinito continua così com'è
interval(1000).pipe(
  skip(5)
).subscribe(console.log);
// 5, 6, 7, 8, ... continua per sempre
```

### Corretto: Combina con take per Impostare Condizione di Terminazione

```ts
import { interval } from 'rxjs';
import { skip, take } from 'rxjs';

// ✅ Esempio corretto: Limita il numero di valori dopo skip
interval(1000).pipe(
  skip(5),
  take(3)
).subscribe({
  next: console.log,
  complete: () => console.log('Completato')
});
// 5, 6, 7, Completato
```


## 🎓 Riepilogo

### Quando Usare skip
- ✅ Quando vuoi ignorare il valore iniziale o i primi N dati
- ✅ Quando vuoi saltare il valore iniziale di BehaviorSubject
- ✅ Quando vuoi ottenere dati per una pagina specifica nella paginazione
- ✅ Quando vuoi saltare il periodo di calibrazione del sensore

### Quando Combinare con take
- ✅ Quando vuoi ottenere solo un range specifico di dati
- ✅ Quando vuoi ottenere la porzione centrale di dati da uno stream infinito

### Note
- ⚠️ Negli stream infiniti, combina con `take` per impostare la condizione di terminazione
- ⚠️ `skip(0)` funziona come lo stream originale (non salta nulla)
- ⚠️ Se il conteggio skip è maggiore del conteggio totale dati, completa senza emettere nulla


## 🚀 Prossimi Passi

- **[take](/it/guide/operators/filtering/take)** - Impara come ottenere i primi N valori
- **[first](/it/guide/operators/filtering/first)** - Impara come ottenere il primo valore o il primo valore che soddisfa una condizione
- **[last](/it/guide/operators/filtering/last)** - Impara come ottenere l'ultimo valore
- **[filter](/it/guide/operators/filtering/filter)** - Impara come filtrare in base a condizioni
- **[Esempi Pratici Operatori di Filtraggio](/it/guide/operators/filtering/practical-use-cases)** - Impara casi d'uso reali
