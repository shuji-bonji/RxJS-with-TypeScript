---
description: "L'operatore expand è un operatore RxJS che crea un nuovo Observable da ogni valore ed espande ricorsivamente il risultato. Implementare modelli di acquisizione ricorsiva dei dati, come l'attraversamento di strutture ad albero, la paginazione delle API, lo scorrimento infinito e la risoluzione delle dipendenze in TypeScript in modo sicuro per il tipo."
---

# espandere - espansione ricorsiva

L'operatore `expand` esegue una trasformazione ricorsiva che **genera un nuovo Observable da ogni valore ed espande anche il risultato**. È ideale per le operazioni che espandono i valori uno dopo l'altro, come l'attraversamento di strutture ad albero, la paginazione API e i calcoli ricorsivi.

## 🔰 Sintassi e utilizzo di base

```ts
import { of } from 'rxjs';
import { expand, take } from 'rxjs';

// 2Processo ricorsivo di raddoppio
of(1).pipe(
  expand(x => of(x * 2)),
  take(5) // Prevenzione del ciclo infinito
).subscribe(console.log);
// Uscita: 1, 2, 4, 8, 16
```

**Flusso dell'operazione**: 1.
1. Viene emesso un valore iniziale di `1`.
La funzione `espandi` riceve `1` e restituisce `of(2)` 3. Viene emesso `2` e viene richiamata la funzione `espandi`.
3. Viene emesso `2` e viene richiamata la funzione `expand` 4. La funzione `expand` riceve `1` e restituisce `of(2)`.
4. La funzione `espandere` riceve `2` e restituisce `of(4)` 5.
5. Questa iterazione...

> [!WARNING]

> Se non viene specificata una condizione di uscita, la funzione `expand` risulterà in un ciclo **infinito**. Assicurarsi di impostare una condizione di uscita, come ad esempio `take` o restituire condizionatamente `EMPTY`.

[🌐 Documentazione ufficiale di RxJS - `expand`](https://rxjs.dev/api/operators/expand)

## 🔄 Differenze con mergeMap

`expand` è simile a `mergeMap`, tranne per il fatto che **i risultati dell'Observable generato vengono elaborati in modo ricorsivo**.

```ts
import { of } from 'rxjs';
import { mergeMap, expand, take } from 'rxjs';

const double = (x: number) => of(x * 2);

// mergeMap: 1Convertire solo una volta
of(1).pipe(
  mergeMap(double),
  take(5)
).subscribe(console.log);
// Uscita: 2
// (1Solo un valore,2non viene convertito di nuovo)

// expand: Convertire ricorsivamente
of(1).pipe(
  expand(double),
  take(5)
).subscribe(console.log);
// Uscita: 1, 2, 4, 8, 16
// (Ogni risultato viene nuovamente trasformato)
```

```ts
import { of } from 'rxjs';
import { expand, take } from 'rxjs';

// 2Processo ricorsivo di raddoppio
of(1).pipe(
  expand(x => of(x * 2)),
  take(5) // Prevenzione del ciclo infinito
).subscribe(console.log);
// Uscita: 1, 2, 4, 8, 16
```

## 💡 Modello tipico di utilizzo

### 1. ricorsione con condizioni di terminazione


```ts
import { of, EMPTY } from 'rxjs';
import { expand } from 'rxjs';

// 10a meno di2raddoppiato
of(1).pipe(
  expand(x => {
    const next = x * 2;
    return next < 10 ? of(next) : EMPTY;
  })
).subscribe(console.log);
// Uscita: 1, 2, 4, 8
// (16viene convertito in10Poiché è maggiore o uguale aEMPTYviene restituito e la fine)
```

### 2. Attraversamento di strutture ad albero

```ts
import { of, from, EMPTY } from 'rxjs';
import { expand, mergeMap } from 'rxjs';

interface TreeNode {
  id: number;
  name: string;
  children?: TreeNode[];
}

const tree: TreeNode = {
  id: 1,
  name: 'Root',
  children: [
    {
      id: 2,
      name: 'Child 1',
      children: [
        { id: 4, name: 'Grandchild 1' },
        { id: 5, name: 'Grandchild 2' }
      ]
    },
    {
      id: 3,
      name: 'Child 2',
      children: [
        { id: 6, name: 'Grandchild 3' }
      ]
    }
  ]
};

// Attraversamento dell'intero albero
of(tree).pipe(
  expand(node =>
    node.children && node.children.length > 0
      ? from(node.children)
      : EMPTY
  )
).subscribe(node => {
  console.log(`ID: ${node.id}, Name: ${node.name}`);
});
// Uscita:
// ID: 1, Name: Root
// ID: 2, Name: Child 1
// ID: 3, Name: Child 2
// ID: 4, Name: Grandchild 1
// ID: 5, Name: Grandchild 2
// ID: 6, Name: Grandchild 3
```

### 3. paginazione dell'API

```ts
import { of, EMPTY } from 'rxjs';
import { expand, mergeMap } from 'rxjs';

interface PageResponse {
  data: string[];
  nextPage: number | null;
}

function fetchPage(page: number): Promise<PageResponse> {
  // APISimulare la richiesta
  return new Promise(resolve => {
    setTimeout(() => {
      if (page > 3) {
        resolve({ data: [], nextPage: null });
      } else {
        resolve({
          data: [`Item ${page}-1`, `Item ${page}-2`, `Item ${page}-3`],
          nextPage: page + 1
        });
      }
    }, 100);
  });
}

// Recuperare tutte le pagine in modo sequenziale
of(1).pipe(
  expand(page => {
    return page > 0 ? of(page) : EMPTY;
  }),
  mergeMap(page => fetchPage(page)),
  expand(response =>
    response.nextPage
      ? of(response.nextPage).pipe(
          mergeMap(nextPage => fetchPage(nextPage))
        )
      : EMPTY
  )
).subscribe(response => {
  console.log(`Dati della pagina:`, response.data);
});
```

#### Implementazione più pratica della paginazione

```ts
import { defer, EMPTY, lastValueFrom } from 'rxjs';
import { expand, map, reduce, tap } from 'rxjs';

interface PaginatedResponse<T> {
  items: T[];
  nextCursor: string | null;
}

function fetchPagedData<T>(
  fetchFn: (cursor: string | null) => Promise<PaginatedResponse<T>>
): Promise<T[]> {
  return lastValueFrom(
    defer(() => fetchFn(null)).pipe(
      expand(response =>
        response.nextCursor
          ? defer(() => fetchFn(response.nextCursor))
          : EMPTY
      ),
      map(response => response.items),
      reduce((acc, items) => [...acc, ...items], [] as T[])
    )
  );
}

// UICreazione di elementi
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Esempio di implementazione della paginazione';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Recuperare tutti i dati';
container.appendChild(button);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.style.padding = '10px';
status.style.backgroundColor = '#f0f0f0';
container.appendChild(status);

const output = document.createElement('pre');
output.style.marginTop = '10px';
output.style.padding = '10px';
output.style.backgroundColor = '#f9f9f9';
output.style.maxHeight = '300px';
output.style.overflow = 'auto';
container.appendChild(output);

// Esempio di utilizzo：FintoAPIOttenere i dati dell'utente con
interface User {
  id: number;
  name: string;
  email: string;
}

// FintoAPISimulare un
async function fetchUsers(cursor: string | null): Promise<PaginatedResponse<User>> {
  // APISimulare le richieste (con100msRitardo)
  await new Promise(resolve => setTimeout(resolve, 100));

  const page = cursor ? parseInt(cursor) : 1;
  const pageSize = 5;
  const totalPages = 4;

  if (page > totalPages) {
    return { items: [], nextCursor: null };
  }

  const items: User[] = Array.from({ length: pageSize }, (_, i) => ({
    id: (page - 1) * pageSize + i + 1,
    name: `User ${(page - 1) * pageSize + i + 1}`,
    email: `user${(page - 1) * pageSize + i + 1}@example.com`
  }));

  return {
    items,
    nextCursor: page < totalPages ? String(page + 1) : null
  };
}

// Acquisizione di tutti i dati con un semplice clic
button.addEventListener('click', async () => {
  button.disabled = true;
  status.textContent = 'Acquisizione dati in corso...';
  output.textContent = '';

  try {
    const allUsers = await fetchPagedData(fetchUsers);

    status.textContent = `Acquisizione completata: ${allUsers.length}Dati utente per`;
    output.textContent = JSON.stringify(allUsers, null, 2);

    console.log(`Numero di tutti gli utenti: ${allUsers.length}`);
    console.log('Dati utente:', allUsers);
  } catch (error) {
    status.textContent = `Errore: ${error}`;
  } finally {
    button.disabled = false;
  }
});
```

## 🧠 Esempio pratico di codice (visualizzazione della gerarchia delle directory)

Questo è un esempio di attraversamento ricorsivo della struttura delle directory di un file system.

```ts
import { of, from, EMPTY } from 'rxjs';
import { expand, tap } from 'rxjs';

interface FileSystemItem {
  name: string;
  type: 'file' | 'directory';
  path: string;
  children?: FileSystemItem[];
  level: number;
}

// Struttura del file system di esempio
const fileSystem: FileSystemItem = {
  name: 'root',
  type: 'directory',
  path: '/root',
  level: 0,
  children: [
    {
      name: 'src',
      type: 'directory',
      path: '/root/src',
      level: 1,
      children: [
        { name: 'index.ts', type: 'file', path: '/root/src/index.ts', level: 2 },
        { name: 'utils.ts', type: 'file', path: '/root/src/utils.ts', level: 2 },
        {
          name: 'components',
          type: 'directory',
          path: '/root/src/components',
          level: 2,
          children: [
            { name: 'Button.tsx', type: 'file', path: '/root/src/components/Button.tsx', level: 3 },
            { name: 'Input.tsx', type: 'file', path: '/root/src/components/Input.tsx', level: 3 }
          ]
        }
      ]
    },
    {
      name: 'docs',
      type: 'directory',
      path: '/root/docs',
      level: 1,
      children: [
        { name: 'README.md', type: 'file', path: '/root/docs/README.md', level: 2 }
      ]
    },
    { name: 'package.json', type: 'file', path: '/root/package.json', level: 1 }
  ]
};

// UICreazione di elementi
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Visualizzazione della gerarchia delle directory';
container.appendChild(title);

const output = document.createElement('pre');
output.style.padding = '10px';
output.style.backgroundColor = '#f5f5f5';
output.style.fontFamily = 'monospace';
output.style.fontSize = '14px';
container.appendChild(output);

const stats = document.createElement('div');
stats.style.marginTop = '10px';
stats.style.padding = '10px';
stats.style.backgroundColor = '#e3f2fd';
container.appendChild(stats);

let fileCount = 0;
let dirCount = 0;

// Struttura di directory espansa ricorsivamente
of(fileSystem).pipe(
  expand(item => {
    if (item.type === 'directory' && item.children && item.children.length > 0) {
      return from(
        item.children.map(child => ({
          ...child,
          level: item.level + 1
        }))
      );
    }
    return EMPTY;
  }),
  tap(item => {
    if (item.type === 'file') {
      fileCount++;
    } else {
      dirCount++;
    }
  })
).subscribe({
  next: item => {
    const indent = '  '.repeat(item.level);
    const icon = item.type === 'directory' ? '📁' : '📄';
    output.textContent += `${indent}${icon} ${item.name}\n`;
  },
  complete: () => {
    stats.textContent = `Numero di directory: ${dirCount}, Numero di file: ${fileCount}`;
  }
});
```

## 📋 Utilizzo sicuro del tipo.

Questo è un esempio di implementazione type-safe in TypeScript che fa uso dei generici.

```ts
import { Observable, of, from, EMPTY } from 'rxjs';
import { expand, filter, take, defaultIfEmpty, reduce } from 'rxjs';

interface Node<T> {
  value: T;
  children?: Node<T>[];
}

class TreeTraversal<T> {
  /**
   * Attraversa la struttura ad albero con una ricerca in larghezza (width-first)
   */
  traverseBFS(root: Node<T>): Observable<Node<T>> {
    return of(root).pipe(
      expand(node =>
        node.children && node.children.length > 0
          ? from(node.children)
          : EMPTY
      )
    );
  }

  /**
   * Cerca il primo nodo che corrisponde ai criteri
   */
  findNode(
    root: Node<T>,
    predicate: (value: T) => boolean
  ): Observable<Node<T> | undefined> {
    return this.traverseBFS(root).pipe(
      filter(node => predicate(node.value)),
      take(1),
      defaultIfEmpty(undefined as Node<T> | undefined)
    );
  }

  /**
   * Conta il numero di tutti i nodi dell'albero
   */
  countNodes(root: Node<T>): Observable<number> {
    return this.traverseBFS(root).pipe(
      reduce((count) => count + 1, 0)
    );
  }

  /**
   * Recupera tutti i nodi con un valore specifico
   */
  findAllNodes(
    root: Node<T>,
    predicate: (value: T) => boolean
  ): Observable<Node<T>[]> {
    return this.traverseBFS(root).pipe(
      filter(node => predicate(node.value)),
      reduce((acc, node) => [...acc, node], [] as Node<T>[])
    );
  }
}

// Esempio di utilizzo
const tree: Node<string> = {
  value: 'A',
  children: [
    {
      value: 'B',
      children: [
        { value: 'D' },
        { value: 'E' }
      ]
    },
    {
      value: 'C',
      children: [
        { value: 'F' }
      ]
    }
  ]
};

const traversal = new TreeTraversal<string>();

// Attraversamento dell'intero albero
traversal.traverseBFS(tree).subscribe(node => {
  console.log(`Visite: ${node.value}`);
});
// Uscita: Visite: A, Visite: B, Visite: C, Visite: D, Visite: E, Visite: F

// Cerca un nodo specifico
traversal.findNode(tree, value => value === 'D').subscribe(node => {
  console.log(`Nodi trovati: ${node?.value}`);
});
// Uscita: Nodi trovati: D

// Conta il numero di nodi
traversal.countNodes(tree).subscribe(count => {
  console.log(`Numero di nodi nell'albero: ${count}`);
});
// Uscita: Numero di nodi nell'albero: 6

// Ottenere tutti i nodi che corrispondono ai criteri
traversal.findAllNodes(tree, value => value.length === 1).subscribe(nodes => {
  console.log(`Nodo a carattere singolo: ${nodes.map(n => n.value).join(', ')}`);
});
// Uscita: Nodo a carattere singolo: A, B, C, D, E, F
```

## 🎯 Combinato con lo scheduler

`expand` funziona in modo sincrono per impostazione predefinita, ma può essere controllato in modo asincrono utilizzando uno scheduler.

```ts
import { of, asyncScheduler } from 'rxjs';
import { expand, take } from 'rxjs';

// Sincrono (predefinito)
console.log('Sincrono (predefinito)expandInizia con');
of(1).pipe(
  expand(x => of(x * 2)),
  take(5)
).subscribe(x => console.log('Sincrono:', x));
console.log('Sincrono (predefinito)expandFine');
// Uscita:
// Sincrono (predefinito)expandInizia con
// Sincrono: 1
// Sincrono: 2
// Sincrono: 4
// Sincrono: 8
// Sincrono: 16
// Sincrono (predefinito)expandFine

// Asincrono (asyncSchedulerUsa)
console.log('AsincronoexpandInizia con');
of(1, asyncScheduler).pipe(
  expand(x => of(x * 2, asyncScheduler)),
  take(5)
).subscribe(x => console.log('Asincrono:', x));
console.log('AsincronoexpandFine');
// Uscita:
// AsincronoexpandInizia con
// AsincronoexpandFine
// Asincrono: 1
// Asincrono: 2
// Asincrono: 4
// Asincrono: 8
// Asincrono: 16
```

> [!TIP]

> Quando si elaborano grandi quantità di dati, lo Scheduler può essere usato per mantenere l'interfaccia utente reattiva senza bloccare il thread principale. Per maggiori informazioni, vedere [Tipi di scheduler e loro utilizzo] (/it/guide/scheduler/tipi).

## 🔄 Esempio di calcolo ricorsivo

### Sequenza di Fibonacci.

```ts
import { of } from 'rxjs';
import { expand, take } from 'rxjs';

// 2Processo ricorsivo di raddoppio
of(1).pipe(
  expand(x => of(x * 2)),
  take(5) // Prevenzione del ciclo infinito
).subscribe(console.log);
// Uscita: 1, 2, 4, 8, 16
```

## Calcoli fattoriali


## ⚠️ Errori comuni

> [!WARNING]

> L'errore più comune con `expand` è dimenticare di impostare una **condizione di uscita** e finire in un ciclo infinito**.

### Errore: nessuna condizione di uscita.


```ts
import { of } from 'rxjs';
import { expand, take } from 'rxjs';

// 2Processo ricorsivo di raddoppio
of(1).pipe(
  expand(x => of(x * 2)),
  take(5) // Prevenzione del ciclo infinito
).subscribe(console.log);
// Uscita: 1, 2, 4, 8, 16
```

### Positivo: con condizione di uscita

{```ts
import { of, EMPTY } from 'rxjs';
import { expand, take, takeWhile } from 'rxjs';

// ✅ Buon esempio1: takeLimita il numero di pezzi in
of(1).pipe(
  expand(x => of(x + 1)),
  take(10)
).subscribe(console.log);

// ✅ Buon esempio2: Restituisce in modo condizionatoEMPTYRestituisce
of(1).pipe(
  expand(x => x < 10 ? of(x + 1) : EMPTY)
).subscribe(console.log);

// ✅ Buon esempio3: takeWhileRestrizioni condizionali con
of(1).pipe(
  expand(x => of(x + 1)),
  takeWhile(x => x <= 10)
).subscribe(console.log);


> [!IMPORTANT]

> I processi ricorsivi dovrebbero sempre rendere esplicita la condizione di uscita e prevenire i loop infiniti, restituendo `take`, `takeWhile` o `EMPTY` a seconda della condizione.

## 🎓 Sommario

### Quando si dovrebbe usare expand.
- ✅ Quando si vuole attraversare ricorsivamente una struttura ad albero o un grafo.
- ✅ Quando si vogliono ottenere tutti i dati con la paginazione API.
- ✅ Se si desidera eseguire calcoli ricorsivi (Fibonacci, fattoriale, ecc.).
- ✅ Se si desidera attraversare strutture di directory o file system
- ✅ Se si desidera esplorare organigrammi o dati gerarchici

### Quando si dovrebbe usare mergeMap.
- ✅ Quando è sufficiente convertire ogni valore una sola volta
- ✅ Normali trasformazioni asincrone che non richiedono un'elaborazione ricorsiva

### Note.
- ⚠️ **Impostare sempre una condizione di uscita** (per evitare loop infiniti).
- ⚠️ Fare attenzione al consumo di memoria (quando si estraggono grandi quantità di dati).
- ⚠️ Poiché funziona in modo sincrono, considerare l'uso di `asyncScheduler` per grandi quantità di dati.
- ⚠️ Poiché il debug è difficile, la registrazione degli stati intermedi con tap è una buona idea.

## 🚀 Prossimi passi.

- **[mergeMap](. /mergeMap)** - imparare le solite trasformazioni asincrone.
- **[switchMap](. /switchMap)** - impara la conversione per passare all'ultimo processo.
- **[concatMap](. /concatMap)** - impara le trasformazioni che vengono eseguite in modo sequenziale.
- **[Tipi di scheduler e loro utilizzo](/it/guide/scheduler/tipi)** - imparare a combinare expand e scheduler.
- **[Esempi pratici di operatori di conversione](/it/guide/schedulatori/tipi)** - imparare a combinare expand e scheduler /casi d'uso pratici)** - imparare a conoscere casi d'uso reali
