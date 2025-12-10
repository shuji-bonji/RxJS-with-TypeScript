---
description: L'operatore groupBy è un operatore di conversione che divide uno stream in più stream raggruppati in base a una chiave specificata. Ogni gruppo viene emesso come un GroupedObservable, permettendo di elaborare stream in parallelo per ciascun gruppo.
titleTemplate: ':title | RxJS'
---

# groupBy - Dividi Stream in Gruppi per Chiave

L'operatore `groupBy` **divide uno stream in più stream (GroupedObservable) in base a una chiave specificata**. Questo permette di elaborare dati in parallelo per ciascun gruppo, abilitando un'efficiente elaborazione di classificazione e aggregazione.

## 🔰 Sintassi e Utilizzo Base

```ts
import { of } from 'rxjs';
import { groupBy, mergeMap, toArray } from 'rxjs';

interface Product {
  category: string;
  name: string;
  price: number;
}

const products: Product[] = [
  { category: 'Alimentari', name: 'Mela', price: 100 },
  { category: 'Bevande', name: 'Acqua', price: 150 },
  { category: 'Alimentari', name: 'Pane', price: 200 },
  { category: 'Bevande', name: 'Caffè', price: 300 },
  { category: 'Alimentari', name: 'Latte', price: 180 },
];

of(...products)
  .pipe(
    groupBy(product => product.category),
    mergeMap(group$ =>
      group$.pipe(
        toArray(),
        // group$.key contiene il valore della chiave di raggruppamento
        mergeMap(items => [{ category: group$.key, items }])
      )
    )
  )
  .subscribe(console.log);

// Output:
// { category: 'Alimentari', items: [{...}, {...}, {...}] }
// { category: 'Bevande', items: [{...}, {...}] }
```

- La funzione `keySelector` restituisce la chiave per ogni elemento.
- Lo stream viene diviso per chiave, e viene emesso un `GroupedObservable` per ogni gruppo.
- L'elaborazione viene applicata a ogni gruppo usando `mergeMap`, `switchMap`, ecc.

[🌐 Documentazione Ufficiale RxJS - `groupBy`](https://rxjs.dev/api/operators/groupBy)

## 💡 Pattern di Utilizzo Tipici

- Classificare e elaborare dati per categoria o tipo
- Calcolare statistiche per gruppo
- Eseguire elaborazioni parallele per utente/dispositivo/regione
- Aggregare log in tempo reale per livello (errore, avviso, informazione)

## 🧠 Esempio di Codice Pratico (con UI)

Questo esempio raggruppa dati di vendita per categoria e visualizza il prezzo totale.

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, reduce } from 'rxjs';

interface Sale {
  category: string;
  product: string;
  price: number;
}

// Crea area di output
const output = document.createElement('div');
output.style.fontFamily = 'monospace';
document.body.appendChild(output);

// Dati di vendita
const sales: Sale[] = [
  { category: 'Elettronica', product: 'Smartphone', price: 80000 },
  { category: 'Alimentari', product: 'Mela', price: 300 },
  { category: 'Elettronica', product: 'Cuffie', price: 15000 },
  { category: 'Abbigliamento', product: 'T-shirt', price: 2500 },
  { category: 'Alimentari', product: 'Latte', price: 200 },
  { category: 'Abbigliamento', product: 'Jeans', price: 8000 },
  { category: 'Elettronica', product: 'Mouse', price: 3000 },
];

from(sales)
  .pipe(
    groupBy(sale => sale.category),
    mergeMap(group$ =>
      group$.pipe(
        reduce(
          (acc, sale) => ({
            category: group$.key,
            total: acc.total + sale.price,
            count: acc.count + 1,
          }),
          { category: '', total: 0, count: 0 }
        )
      )
    )
  )
  .subscribe(result => {
    const div = document.createElement('div');
    div.innerHTML = `
      <strong>${result.category}</strong>:
      ${result.count} articoli,
      Totale ¥${result.total.toLocaleString()}
    `;
    output.appendChild(div);
  });
```

## 🎯 Esempio di Classificazione Log per Livello

Questo è un esempio pratico di raggruppamento log per livello.

```ts
import { from, interval } from 'rxjs';
import { groupBy, mergeMap, map, take, toArray } from 'rxjs';

interface LogEntry {
  level: 'info' | 'warn' | 'error';
  message: string;
  timestamp: number;
}

// Genera dati log fittizi
const logs$ = interval(100).pipe(
  take(20),
  map(i => {
    const levels: LogEntry['level'][] = ['info', 'warn', 'error'];
    const level = levels[Math.floor(Math.random() * 3)];
    return {
      level,
      message: `Messaggio ${i}`,
      timestamp: Date.now(),
    } as LogEntry;
  })
);

logs$
  .pipe(
    groupBy(log => log.level),
    mergeMap(group$ =>
      group$.pipe(
        toArray(),
        map(logs => ({
          level: group$.key,
          logs,
          count: logs.length,
        }))
      )
    )
  )
  .subscribe(result => {
    const icon =
      result.level === 'error' ? '🔴' : result.level === 'warn' ? '🟡' : '🟢';
    console.log(`${icon} ${result.level.toUpperCase()}: ${result.count} voci`);
    result.logs.forEach(log => console.log(`   - ${log.message}`));
  });
```

## 🎯 Utilizzo di groupBy Type-Safe

Questo è un esempio di utilizzo dell'inferenza dei tipi di TypeScript.

```ts
import { from } from 'rxjs';
import { groupBy, mergeMap, toArray, map } from 'rxjs';

interface User {
  id: number;
  name: string;
  department: 'Vendite' | 'Sviluppo' | 'Risorse Umane';
  salary: number;
}

const users: User[] = [
  { id: 1, name: 'Mario', department: 'Vendite', salary: 400000 },
  { id: 2, name: 'Luigi', department: 'Sviluppo', salary: 500000 },
  { id: 3, name: 'Chiara', department: 'Vendite', salary: 450000 },
  { id: 4, name: 'Giuseppe', department: 'Sviluppo', salary: 550000 },
  { id: 5, name: 'Elena', department: 'Risorse Umane', salary: 350000 },
];

from(users)
  .pipe(
    // Raggruppa per dipartimento
    groupBy<User, User['department']>(user => user.department),
    mergeMap(group$ =>
      group$.pipe(
        toArray(),
        map(members => ({
          department: group$.key,
          members,
          averageSalary:
            members.reduce((sum, u) => sum + u.salary, 0) / members.length,
        }))
      )
    )
  )
  .subscribe(result => {
    console.log(`=== ${result.department} ===`);
    console.log(`Membri: ${result.members.map(u => u.name).join(', ')}`);
    console.log(`Stipendio medio: ¥${result.averageSalary.toLocaleString()}`);
    console.log('');
  });
```

## 🔍 Funzione elementSelector (Opzionale)

Puoi specificare un secondo argomento `elementSelector` per trasformare gli elementi all'interno di ogni gruppo.

```ts
import { of } from 'rxjs';
import { groupBy, mergeMap, toArray } from 'rxjs';

interface Product {
  category: string;
  name: string;
  price: number;
}

const products: Product[] = [
  { category: 'Frutta', name: 'Mela', price: 100 },
  { category: 'Frutta', name: 'Banana', price: 150 },
  { category: 'Verdura', name: 'Carota', price: 80 },
];

of(...products)
  .pipe(
    groupBy(
      p => p.category,
      p => p.name // Estrai solo il nome
    ),
    mergeMap(group$ =>
      group$.pipe(
        toArray(),
        mergeMap(names => [{ category: group$.key, names }])
      )
    )
  )
  .subscribe(console.log);

// Output:
// { category: 'Frutta', names: ['Mela', 'Banana'] }
// { category: 'Verdura', names: ['Carota'] }
```

## 🎯 Esempio di Aggregazione Dati in Tempo Reale

Questo è un esempio di aggregazione di dati di transazione in tempo reale per tipo.

```ts
import { interval } from 'rxjs';
import { groupBy, mergeMap, scan, map, take } from 'rxjs';

interface Transaction {
  type: 'acquisto' | 'vendita' | 'trasferimento';
  amount: number;
  timestamp: number;
}

// Genera transazioni fittizie
const transactions$ = interval(200).pipe(
  take(30),
  map(i => {
    const types: Transaction['type'][] = ['acquisto', 'vendita', 'trasferimento'];
    return {
      type: types[Math.floor(Math.random() * 3)],
      amount: Math.floor(Math.random() * 10000) + 1000,
      timestamp: Date.now(),
    } as Transaction;
  })
);

transactions$
  .pipe(
    groupBy(tx => tx.type),
    mergeMap(group$ =>
      group$.pipe(
        scan(
          (acc, tx) => ({
            type: tx.type,
            count: acc.count + 1,
            total: acc.total + tx.amount,
            average: (acc.total + tx.amount) / (acc.count + 1),
          }),
          { type: group$.key, count: 0, total: 0, average: 0 }
        )
      )
    )
  )
  .subscribe(stats => {
    console.log(
      `[${stats.type}] Conteggio: ${stats.count}, ` +
        `Totale: ¥${stats.total.toLocaleString()}, ` +
        `Media: ¥${Math.round(stats.average).toLocaleString()}`
    );
  });
```

## ⚠️ Note

### 1. Gestione Subscription dei Gruppi

Ogni `GroupedObservable` è uno stream indipendente, quindi deve essere sottoscritto. Se non ti iscrivi, i dati di quel gruppo vengono scartati.

```ts
// ❌ Esempio sbagliato: Non ti sottoscrivi a ogni gruppo
source$.pipe(groupBy(keySelector)).subscribe(group$ => {
  console.log('Chiave gruppo:', group$.key);
  // I dati dentro il gruppo vengono ignorati
});

// ✅ Esempio corretto: Elabora ogni gruppo
source$
  .pipe(
    groupBy(keySelector),
    mergeMap(group$ => group$.pipe(toArray()))
  )
  .subscribe(result => {
    console.log(result);
  });
```

### 2. Attenzione ai Memory Leak

Se lo stream continua indefinitamente e vengono create nuove chiavi, il numero di gruppi cresce illimitatamente, rischiando perdite di memoria.

```ts
// ⚠️ Richiede attenzione: Le nuove chiavi aumentano indefinitamente
userActions$.pipe(
  groupBy(action => action.sessionId) // Nuova chiave per ogni sessione
);

// ✅ Contromisura: Rilascia i gruppi quando necessario
userActions$.pipe(
  groupBy(
    action => action.sessionId,
    undefined,
    group$ => group$.pipe(timeout(30000)) // Rilascia il gruppo dopo il timeout
  )
);
```

### 3. Utilizzo Memoria

Poiché `groupBy` mantiene più gruppi simultaneamente, bisogna considerare l'utilizzo di memoria quando ci sono molti tipi di chiave.

## 🆚 Confronto con altri Operatori di Raggruppamento

| Operatore | Metodo di Output | Timing di Output | Caso d'Uso |
|:---|:---|:---|:---|
| `groupBy` | `GroupedObservable` per ogni gruppo | Durante lo stream | Elaborazione streaming per gruppo |
| `reduce` | Un valore singolo | Al completamento | Aggregazione finale |
| `scan` | Aggiorna ogni volta | Per ogni valore | Aggiornamento stato cumulativo |
| `toArray` | Array singolo | Al completamento | Raccolta in array |

## 📚 Operatori Correlati

- [`mergeMap`](/it/guide/operators/transformation/mergeMap) - Elabora stream interni in parallelo
- [`reduce`](/it/guide/operators/transformation/reduce) - Riduci stream a un singolo valore
- [`scan`](/it/guide/operators/transformation/scan) - Accumula stato mantenendo risultati intermedi
- [`partition`](/it/guide/creation-functions/selection-partition/partition) - Dividi stream in due con condizioni booleane

## Riepilogo

L'operatore `groupBy` **divide uno stream in più gruppi in base a una chiave specificata**. Ogni gruppo viene emesso come un `GroupedObservable`, permettendo diverse elaborazioni per gruppo. È adatto per casi d'uso come classificazione dati, aggregazione per gruppo e elaborazione parallela. Tuttavia, con stream infiniti e molti tipi di chiave, c'è rischio di perdite di memoria, quindi l'impostazione di condizioni di rilascio appropriati è importante.
