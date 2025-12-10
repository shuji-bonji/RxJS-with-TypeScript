---
description: expand è un operatore RxJS per elaborare ricorsivamente Observable. Applica una funzione di proiezione ai valori emessi e riapplica la funzione ai risultati, continuando l'elaborazione come un'espansione. È ideale per attraversamento alberi, impaginazione e caricamento dati gerarchici.
titleTemplate: ':title | RxJS'
---

# expand - Espandi Stream Ricorsivamente

L'operatore `expand` **applica una funzione di proiezione ai valori emessi, e riapplica la stessa funzione ai risultati, espandendo lo stream ricorsivamente**. Questo abilita l'elaborazione simile alla ricorsione per attraversamento di strutture ad albero e caricamento di dati impaginati.

## 🔰 Sintassi e Utilizzo Base

```ts
import { of } from 'rxjs';
import { expand, take } from 'rxjs';

// Raddoppia il valore
of(1)
  .pipe(
    expand(value => of(value * 2)),
    take(5)
  )
  .subscribe(console.log);

// Output:
// 1
// 2
// 4
// 8
// 16
```

- Il primo valore `1` viene emesso così com'è.
- La funzione di proiezione restituisce `1 * 2 = 2`, che viene emesso.
- Questo viene ripetuto (`4`, `8`, `16`...).
- Senza `take(5)`, continuerà indefinitamente.

[🌐 Documentazione Ufficiale RxJS - `expand`](https://rxjs.dev/api/operators/expand)

## 💡 Pattern di Utilizzo Tipici

- Caricamento dati impaginati (API con scroll infinito)
- Attraversamento di strutture ad albero/grafi
- Raccolta dati gerarchia organizzativa/raggruppamento
- Gestione redirect (seguendo URL)

## 🧠 Esempio di Codice Pratico 1: Caricamento Dati Impaginati

Questo esempio simula un'API impaginata.

```ts
import { of, Observable, EMPTY } from 'rxjs';
import { expand, map, delay, reduce } from 'rxjs';

interface Page<T> {
  data: T[];
  nextPage: number | null;
}

// API fittizia (impaginata)
function fetchPage(pageNumber: number): Observable<Page<string>> {
  const totalPages = 5;

  return of({
    data: [`Elemento ${pageNumber}-1`, `Elemento ${pageNumber}-2`, `Elemento ${pageNumber}-3`],
    nextPage: pageNumber < totalPages ? pageNumber + 1 : null,
  }).pipe(delay(300)); // Simula ritardo di rete
}

console.log('Avvio caricamento dati impaginati...');

// Carica dalla prima pagina
fetchPage(1)
  .pipe(
    expand(page => (page.nextPage !== null ? fetchPage(page.nextPage) : EMPTY)),
    map(page => page.data),
    reduce((acc, data) => [...acc, ...data], [] as string[])
  )
  .subscribe(allData => {
    console.log('Tutti i dati:', allData);
    console.log('Totale elementi:', allData.length);
  });

// Output:
// Avvio caricamento dati impaginati...
// Tutti i dati: ['Elemento 1-1', 'Elemento 1-2', 'Elemento 1-3', 'Elemento 2-1', ...]
// Totale elementi: 15
```

**Punti Chiave**:
- `EMPTY` restituito quando non c'è pagina successiva per fermare la ricorsione
- `reduce` raccoglie i dati di tutte le pagine insieme

## 🎯 Esempio di Codice Pratico 2: Attraversamento Struttura ad Albero

Questo esempio attraversa tutti i nodi di una struttura ad albero.

```ts
import { of, from, EMPTY } from 'rxjs';
import { expand, mergeMap, toArray } from 'rxjs';

interface TreeNode {
  id: number;
  name: string;
  children?: TreeNode[];
}

const tree: TreeNode = {
  id: 1,
  name: 'Radice',
  children: [
    {
      id: 2,
      name: 'Figlio 1',
      children: [
        { id: 4, name: 'Nipote 1' },
        { id: 5, name: 'Nipote 2' },
      ],
    },
    {
      id: 3,
      name: 'Figlio 2',
      children: [{ id: 6, name: 'Nipote 3' }],
    },
  ],
};

// Attraversa tutti i nodi dell'albero
of(tree)
  .pipe(
    expand(node =>
      node.children && node.children.length > 0 ? from(node.children) : EMPTY
    ),
    toArray()
  )
  .subscribe(allNodes => {
    console.log('Tutti i nodi:');
    allNodes.forEach(node => {
      console.log(`  ID: ${node.id}, Nome: ${node.name}`);
    });
  });

// Output:
// Tutti i nodi:
//   ID: 1, Nome: Radice
//   ID: 2, Nome: Figlio 1
//   ID: 3, Nome: Figlio 2
//   ID: 4, Nome: Nipote 1
//   ID: 5, Nome: Nipote 2
//   ID: 6, Nome: Nipote 3
```

## 🎯 Esempio Pratico: Raccolta Gerarchia Organizzativa

Questo esempio raccoglie tutti i membri sotto un manager in una gerarchia organizzativa.

```ts
import { of, from, EMPTY, Observable } from 'rxjs';
import { expand, mergeMap, toArray, delay, map } from 'rxjs';

interface Employee {
  id: number;
  name: string;
  managerId: number | null;
}

// Dati dipendenti
const employees: Employee[] = [
  { id: 1, name: 'CEO', managerId: null },
  { id: 2, name: 'CTO', managerId: 1 },
  { id: 3, name: 'CFO', managerId: 1 },
  { id: 4, name: 'Responsabile Sviluppo', managerId: 2 },
  { id: 5, name: 'Responsabile QA', managerId: 2 },
  { id: 6, name: 'Ingegnere Senior', managerId: 4 },
  { id: 7, name: 'Ingegnere Junior', managerId: 4 },
  { id: 8, name: 'Ingegnere QA', managerId: 5 },
];

// API fittizia (ottieni subordinati)
function getSubordinates(managerId: number): Observable<Employee[]> {
  const subordinates = employees.filter(e => e.managerId === managerId);
  return of(subordinates).pipe(delay(100)); // Simula ritardo API
}

// Ottieni tutti i membri da un manager specifico
function getAllTeamMembers(managerId: number): Observable<Employee[]> {
  return getSubordinates(managerId).pipe(
    expand(subordinates =>
      subordinates.length > 0
        ? from(subordinates).pipe(
            mergeMap(sub => getSubordinates(sub.id))
          )
        : EMPTY
    ),
    mergeMap(subs => from(subs)),
    toArray()
  );
}

// Ottieni tutti i subordinati del CTO
console.log('Membri del team del CTO:');
getAllTeamMembers(2).subscribe(members => {
  members.forEach(m => console.log(`  - ${m.name} (ID: ${m.id})`));
  console.log(`Totale: ${members.length} persone`);
});

// Output:
// Membri del team del CTO:
//   - Responsabile Sviluppo (ID: 4)
//   - Responsabile QA (ID: 5)
//   - Ingegnere Senior (ID: 6)
//   - Ingegnere Junior (ID: 7)
//   - Ingegnere QA (ID: 8)
// Totale: 5 persone
```

## 🔍 Controllo della Concorrenza

L'operatore `expand` può controllare il numero di elaborazioni parallele con il secondo argomento `concurrent`.

```ts
import { of } from 'rxjs';
import { expand, delay, take } from 'rxjs';

// Controlla la concorrenza
of(1)
  .pipe(
    expand(
      value => of(value + 1).pipe(delay(100)),
      2 // Massimo 2 elaborazioni parallele
    ),
    take(10)
  )
  .subscribe(console.log);
```

**Casi d'uso**:
- Controllare il carico su API esterne
- Prevenire errori di rate limiting
- Gestire le limitazioni di risorse

## ⚠️ Note

### 1. Condizione di Fine Obbligatoria

`expand` continua la ricorsione indefinitamente, quindi una condizione di fine è essenziale.

```ts
// ❌ Esempio sbagliato: Nessuna condizione di fine
of(1)
  .pipe(expand(value => of(value + 1)))
  .subscribe(); // Loop infinito!

// ✅ Esempio corretto 1: Usa take per impostare la condizione di fine
of(1)
  .pipe(
    expand(value => of(value + 1)),
    take(10)
  )
  .subscribe();

// ✅ Esempio corretto 2: Ferma la ricorsione nella funzione di proiezione
of(1)
  .pipe(expand(value => (value < 100 ? of(value * 2) : EMPTY)))
  .subscribe();
```

### 2. Gestione Errori

Se un errore si verifica a metà della ricorsione, l'intero stream viene interrotto. Imposta una gestione errori appropriata.

```ts
import { of, EMPTY, catchError } from 'rxjs';
import { expand } from 'rxjs';

of(1)
  .pipe(
    expand(value =>
      fetchData(value).pipe(
        catchError(err => {
          console.error('Errore:', err);
          return EMPTY; // Interrompi elaborazione in caso di errore
        })
      )
    )
  )
  .subscribe();
```

### 3. Attenzione all'Utilizzo Memoria

Quando si espande una struttura dati grande, tutti i dati intermedi vengono mantenuti in memoria. Per grandi quantità di dati, considera l'elaborazione in batch.

```ts
// ⚠️ Raccogliere milioni di record in memoria potrebbe essere problematico
fetchAllPages()
  .pipe(
    expand(/* ... */),
    toArray() // Tutti i dati in memoria
  )
  .subscribe();

// ✅ Contromisura: Elabora elemento per elemento
fetchAllPages()
  .pipe(expand(/* ... */))
  .subscribe(item => {
    processItem(item); // Elabora elemento per elemento
  });
```

## 🆚 Confronto con mergeMap

| Operatore | Comportamento | Caso d'Uso |
|:---|:---|:---|
| `expand` | Applica ricorsivamente la funzione di proiezione al risultato | Attraversamento alberi, impaginazione |
| `mergeMap` | Applica la funzione di proiezione una volta e unisce | Elaborazione parallela di Observable interni |

```ts
import { of, from } from 'rxjs';
import { expand, mergeMap, take } from 'rxjs';

// expand: Elaborazione ricorsiva
of(1)
  .pipe(
    expand(x => of(x * 2)),
    take(5)
  )
  .subscribe(console.log);
// Output: 1, 2, 4, 8, 16

// mergeMap: Elaborazione una tantum
of(1, 2, 3)
  .pipe(mergeMap(x => of(x * 10)))
  .subscribe(console.log);
// Output: 10, 20, 30
```

## 📚 Operatori Correlati

- [`mergeMap`](/it/guide/operators/transformation/mergeMap) - Converti ed espandi Observable interni
- [`concatMap`](/it/guide/operators/transformation/concatMap) - Elabora Observable interni sequenzialmente
- [`reduce`](/it/guide/operators/transformation/reduce) - Aggrega risultati
- [`scan`](/it/guide/operators/transformation/scan) - Emetti risultati intermedi di aggregazione

## Riepilogo

L'operatore `expand` è un operatore per **espandere stream ricorsivamente**. È adatto per casi d'uso come impaginazione, attraversamento di strutture ad albero e attraversamento di dati gerarchici. Tuttavia, senza condizione di fine funzionerà indefinitamente, quindi imposta sempre una condizione di uscita appropriata con `take` o `EMPTY`. Inoltre, per grandi quantità di dati, considera l'utilizzo memoria ed elabora elemento per elemento.
