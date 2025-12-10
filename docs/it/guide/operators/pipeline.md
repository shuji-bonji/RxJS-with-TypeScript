---
description: "Impara la costruzione della pipeline RxJS con il metodo pipe: Trasforma, filtra e combina flussi di dati in modo dichiarativo con esempi completi di concatenamento operatori"
---

# Cos'è la Pipeline RxJS?

Il pipelining in RxJS è un meccanismo per applicare una serie di operazioni (operatori) a un Observable in sequenza. Il pipelining ti permette di trasformare, filtrare e combinare flussi di dati in più fasi, permettendoti di controllare il flusso dei dati in uno stile di programmazione dichiarativo.

## Struttura Base di una Pipeline

[📘 RxJS Ufficiale: pipe()](https://rxjs.dev/api/index/function/pipe)

Il metodo `pipe()` di RxJS viene usato per costruire una pipeline. La sintassi è la seguente.

```ts
import { Observable } from 'rxjs';
import { map, filter, tap } from 'rxjs';

const source$: Observable<number> = // Qualche Observable
source$.pipe(
  // Concatena più operatori
  operator1(),
  operator2(),
  operator3(),
  // ...
).subscribe(value => {
  // Elabora il risultato
});
```

## Esempi Pratici

### Conversione Dati Base

```ts
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

// Stream di numeri
const numbers$ = of(1, 2, 3, 4, 5);

// Costruisci una pipeline
numbers$.pipe(
  // Passa solo numeri pari
  filter(n => n % 2 === 0),
  // Raddoppia il valore
  map(n => n * 2)
).subscribe(
  value => console.log(`Risultato: ${value}`)
);

// Output:
// Risultato: 4
// Risultato: 8
```

### Elaborazione Dati Complessa

```ts
import { fromEvent, map, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

type User = {
  id: number;
  name: string;
  username: string;
  email: string;
};
type Post = {
  userId: number;
  id: number;
  title: string;
  body: string;
};

// Crea elementi DOM
const searchButton = document.createElement('button');
searchButton.innerText = 'Cerca';
document.body.appendChild(searchButton);

const resultBox = document.createElement('div');
resultBox.id = 'results';
document.body.appendChild(resultBox);

// Richiesta API al click del bottone
fromEvent(searchButton, 'click')
  .pipe(
    switchMap(() =>
      // Prima chiamata API
      ajax.getJSON<User>('https://jsonplaceholder.typicode.com/users/1').pipe(
        // Seconda chiamata API per ottenere i post dell'utente
        switchMap((user) => {
          const header = document.createElement('h3');
          header.textContent = `Utente: ${user.name}`;
          resultBox.innerHTML = ''; // Pulisci risultati precedenti
          resultBox.appendChild(header);

          return ajax.getJSON<Post[]>(
            `https://jsonplaceholder.typicode.com/posts?userId=${user.id}`
          );
        }),
        // Ottieni solo i primi 3 post
        map((posts) => posts.slice(0, 3))
      )
    )
  )
  .subscribe((posts) => {
    // Visualizza post sullo schermo
    resultBox.innerHTML += '<h4>Post:</h4>';
    posts.forEach((post) => {
      const div = document.createElement('div');
      div.innerHTML = `<strong>${post.title}</strong><p>${post.body}</p>`;
      resultBox.appendChild(div);
    });
  });

```


## Vantaggi della Pipeline

Prima, guardiamo il codice scritto in modo imperativo. Come mostrato dopo, il pipelining RxJS ti permette di riscriverlo in una forma più leggibile e manutenibile rendendo chiaro l'intento del processo.

### 1. Migliore Leggibilità e Manutenibilità

```ts
// Elaborazione in stile imperativo
const data = [
  { id: 3, active: true },
  { id: 1, active: false },
  { id: 2, active: true }
];

const activeItems = [];
for (const item of data) {
  if (item.active) {
    activeItems.push({ ...item, label: `Elemento #${item.id}` });
  }
}
activeItems.sort((a, b) => a.id - b.id);

const div1 = document.createElement('div');
div1.innerHTML = '<h3>Stile Imperativo</h3>';
activeItems.forEach(item => {
  const p = document.createElement('p');
  p.textContent = item.label;
  div1.appendChild(p);
});
document.body.appendChild(div1);
```
⬇️⬇️⬇️
```ts
import { of } from 'rxjs';
import { filter, map, toArray } from 'rxjs';

const output = document.createElement('div');
output.innerHTML = '<h3>Migliore Leggibilità e Manutenibilità</h3>';
document.body.appendChild(output);

of(
  { id: 3, active: true },
  { id: 1, active: false },
  { id: 2, active: true }
).pipe(
  filter(item => item.active),
  map(item => ({ ...item, label: `Elemento #${item.id}` })),
  toArray(),
  map(array => array.sort((a, b) => a.id - b.id))
).subscribe(sorted => {
  sorted.forEach(item => {
    const div = document.createElement('div');
    div.textContent = item.label;
    output.appendChild(div);
  });
});
```

Il pipelining rende chiaro il flusso dei dati ed elimina la necessità di riassegnare variabili o gestire stati intermedi.



Codice procedurale come quello sopra può essere scritto in modo conciso in stile dichiarativo usando il pipelining RxJS. Un esempio è mostrato sotto.

### 2. Stile di Programmazione Dichiarativo

Il pipelining promuove uno stile dichiarativo che indica esplicitamente "cosa fare". Questo rende l'intento del codice più chiaro.

```ts
// Elaborazione in stile procedurale
const usersList = [
  { status: 'active', firstName: 'Mario', lastName: 'Rossi', email: 'mario@example.com' },
  { status: 'inactive', firstName: 'Anna', lastName: 'Rossi', email: 'anna@example.com' },
  { status: 'active', firstName: 'Giovanni', lastName: 'Bianchi', email: 'giovanni@example.com' }
];

const activeUsers2 = [];
for (const user of usersList) {
  if (user.status === 'active') {
    const name = `${user.firstName} ${user.lastName}`;
    activeUsers2.push({ name, email: user.email });
  }
}

const div2 = document.createElement('div');
div2.innerHTML = '<h3>Stile Procedurale</h3>';
activeUsers2.forEach(user => {
  const p = document.createElement('p');
  p.textContent = `${user.name} (${user.email})`;
  div2.appendChild(p);
});
document.body.appendChild(div2);
```

⬇️⬇️⬇️

```ts
// Stile di programmazione dichiarativo
import { from } from 'rxjs';
import { filter, map } from 'rxjs';

const out2 = document.createElement('div');
out2.innerHTML = '<h3>Stile Dichiarativo</h3>';
document.body.appendChild(out2);

const users = [
  { status: 'active', firstName: 'Mario', lastName: 'Rossi', email: 'mario@example.com' },
  { status: 'inactive', firstName: 'Anna', lastName: 'Rossi', email: 'anna@example.com' },
  { status: 'active', firstName: 'Giovanni', lastName: 'Bianchi', email: 'giovanni@example.com' }
];

from(users).pipe(
  filter(user => user.status === 'active'),
  map(user => ({
    name: `${user.firstName} ${user.lastName}`,
    email: user.email
  }))
).subscribe(user => {
  const div = document.createElement('div');
  div.textContent = `${user.name} (${user.email})`;
  out2.appendChild(div);
});
```


Similmente qui, prendiamo codice che descrive l'elaborazione in modo procedurale e riorganizziamolo con il pipelining. Elaborazioni complesse possono essere semplicemente costruite componendo singoli operatori.

### 3. Componibilità

Il pipelining ti permette di costruire elaborazioni complesse combinando piccole operazioni.

```ts
// Elaborazione in stile procedurale (imperativo)
const rawUsers = [
  { firstName: 'Alice', lastName: 'Smith', status: 'active' },
  { firstName: 'Bob', lastName: 'Brown', status: 'inactive' },
  { firstName: 'Carol', lastName: 'Jones', status: 'active' }
];

const activeUsers = [];
for (const user of rawUsers) {
  if (user.status === 'active') {
    const fullName = `${user.firstName} ${user.lastName}`;
    activeUsers.push({ ...user, fullName });
  }
}
activeUsers.sort((a, b) => a.fullName.localeCompare(b.fullName));

const div0 = document.createElement('div');
div0.innerHTML = '<h3>Stile Procedurale</h3>';
activeUsers.forEach(user => {
  const p = document.createElement('p');
  p.textContent = user.fullName;
  div0.appendChild(p);
});
document.body.appendChild(div0);
```

⬇️⬇️⬇️

```ts
// Stile di programmazione dichiarativo
import { from } from 'rxjs';
import { filter, map, toArray } from 'rxjs';

const out3 = document.createElement('div');
out3.innerHTML = '<h3>Componibilità</h3>';
document.body.appendChild(out3);

const users3 = [
  { firstName: 'Alice', lastName: 'Smith', status: 'active' },
  { firstName: 'Bob', lastName: 'Brown', status: 'inactive' },
  { firstName: 'Carol', lastName: 'Jones', status: 'active' }
];

const filterActive = filter((user: any) => user.status === 'active');
const formatFullName = map((user: any) => ({ ...user, fullName: `${user.firstName} ${user.lastName}` }));
const collectAndSort = toArray();
const sortByName = map((users: any[]) => users.sort((a, b) => a.fullName.localeCompare(b.fullName)));

from(users3).pipe(
  filterActive,
  formatFullName,
  collectAndSort,
  sortByName
).subscribe(users => {
  users.forEach(user => {
    const div = document.createElement('div');
    div.textContent = user.fullName;
    out3.appendChild(div);
  });
});
```

## Tecniche di Ottimizzazione della Pipeline

### 1. Importanza dell'Ordine degli Operatori

L'ordine degli operatori ha un impatto significativo sia sulle performance che sulla funzionalità.

```ts
// Inefficiente: map viene applicato a tutti gli elementi
observable$.pipe(
  map(x => expensiveTransformation(x)),
  filter(x => x > 10)
)

// Efficiente: filter viene eseguito prima, riducendo gli elementi da trasformare
observable$.pipe(
  filter(x => x > 10),
  map(x => expensiveTransformation(x))
)
```

### 2. Creare Pipeline Personalizzate

Elaborazioni complesse possono essere estratte in pipeline riutilizzabili.

```ts
import { Observable, pipe } from 'rxjs';
import { filter, map } from 'rxjs';

// Funzione pipeline personalizzata
export function filterAndTransform<T, R>(
  filterFn: (value: T) => boolean,
  transformFn: (value: T) => R
) {
  return pipe(
    filter(filterFn),
    map(transformFn)
  );
}

// Esempio di utilizzo
observable$.pipe(
  filterAndTransform(
    x => x > 10,
    x => x * 2
  )
).subscribe(console.log);
```

## Errori Comuni con le Pipeline

### 1. Ordine Errato degli Operatori

```ts
// ❌ Se applichi filter prima di debounceTime,
// filter verrà eseguito per ogni input, riducendo l'effetto del debounce
inputEvents$.pipe(
  filter(text => text.length > 2),
  debounceTime(300)
)

// ✅ Applica debounceTime prima
inputEvents$.pipe(
  debounceTime(300),
  filter(text => text.length > 2)
)
```

### 2. Effetti Collaterali nella Pipeline

```ts
// ❌ Esegui direttamente effetti collaterali nella pipeline
observable$.pipe(
  map(data => {
    // Effetti collaterali (esempio cattivo)
    console.log(data);
    localStorage.setItem('lastData', JSON.stringify(data));
    return data;
  })
)

// ✅ Usa l'operatore tap
observable$.pipe(
  tap(data => {
    console.log(data);
    localStorage.setItem('lastData', JSON.stringify(data));
  }),
  // Esegui trasformazione dati con map
  map(data => transformData(data))
)
```

## Riepilogo

Le pipeline RxJS sono un potente meccanismo per gestire flussi di dati asincroni complessi in modo dichiarativo e componibile. Pipeline progettate correttamente possono migliorare notevolmente la leggibilità, manutenibilità e riutilizzabilità del codice.

Quando progetti pipeline, è buona pratica tenere a mente i seguenti punti:

1. Scegli la sequenza più efficiente di operatori
2. Estrai e riutilizza pattern di pipeline comuni
3. Isola gli effetti collaterali con operatori `tap`
4. Assicurati che ogni step nella pipeline abbia una singola responsabilità

Tale approccio orientato alla pipeline è particolarmente potente in scenari come elaborazione complessa di eventi UI, richieste API e gestione stato.
