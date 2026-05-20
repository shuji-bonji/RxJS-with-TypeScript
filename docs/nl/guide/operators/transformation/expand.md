---
description: "De expand operator is een RxJS operator die van elke waarde een nieuwe Observable maakt en het resultaat recursief uitbreidt. Implementeer recursieve gegevensverwervingspatronen zoals boomstructuur traversal, API pagination, infinite scrolling, dependency resolution, etc. in TypeScript op een type-veilige manier."
---

# uitbreiden - recursief uitbreiden

De `expand` operator voert een recursieve transformatie uit die **van elke waarde een nieuwe Observable genereert en het resultaat ook uitbreidt**. Het is ideaal voor bewerkingen die waarden één voor één uitbreiden, zoals het doorlopen van boomstructuren, API-paginatie en recursieve berekeningen.

## 🔰 Basis syntaxis en gebruik

```ts
import { of } from 'rxjs';
import { expand, take } from 'rxjs';

// 2Recursief proces van verdubbelen
of(1).pipe(
  expand(x => of(x * 2)),
  take(5) // Preventie van oneindige lus
).subscribe(console.log);
// Uitvoer: 1, 2, 4, 8, 16
```

**Stroom van de bewerking**: 1.
1. een initiële waarde van `1` wordt uitgegeven
2. De functie `expand` ontvangt `1` en geeft `of(2)` terug 3. De functie `expand` wordt aangeroepen.
3. `2` wordt uitgegeven en de functie `expand` wordt opnieuw aangeroepen 4. De functie `expand` ontvangt `1` en geeft `of(2)` terug
4. De functie `expand` ontvangt `2` en geeft `of(4)` terug 5. De functie `expand` wordt opnieuw aangeroepen.
5. deze iteratie...

```ts
import { of } from 'rxjs';
import { expand, take } from 'rxjs';

// 2Recursief proces van verdubbelen
of(1).pipe(
  expand(x => of(x * 2)),
  take(5) // Preventie van oneindige lus
).subscribe(console.log);
// Uitvoer: 1, 2, 4, 8, 16
```

> `uitbreiden` resulteert in een **oneindige lus** als er geen afsluitvoorwaarde is opgegeven. Zorg ervoor dat je een afsluitvoorwaarde instelt, zoals `take` of voorwaardelijk `EMPTY` retourneren.

[🌐 Officiële RxJS documentatie - `uitbreiden`](https://rxjs.dev/api/operators/expand)

## Verschillen met mergeMap

`expand` is vergelijkbaar met `mergeMap`, behalve dat **de resultaten van de gegenereerde Observable ook recursief worden verwerkt**.


```ts
import { of } from 'rxjs';
import { mergeMap, expand, take } from 'rxjs';

const double = (x: number) => of(x * 2);

// mergeMap: 1Slechts eenmaal converteren
of(1).pipe(
  mergeMap(double),
  take(5)
).subscribe(console.log);
// Uitvoer: 2
// (1Slechts één waarde,2wordt niet opnieuw geconverteerd)

// expand: Recursief converteren
of(1).pipe(
  expand(double),
  take(5)
).subscribe(console.log);
// Uitvoer: 1, 2, 4, 8, 16
// (Elk resultaat wordt opnieuw omgezet)
```

TABEL 14

## 💡 Typisch gebruikspatroon

### 1. recursie met beëindigingsvoorwaarden

```ts
import { of, EMPTY } from 'rxjs';
import { expand } from 'rxjs';

// 10naar minder dan2verdubbeld
of(1).pipe(
  expand(x => {
    const next = x * 2;
    return next < 10 ? of(next) : EMPTY;
  })
).subscribe(console.log);
// Uitvoer: 1, 2, 4, 8
// (16wordt geconverteerd naar10Aangezien het groter is dan of gelijk aanEMPTYwordt teruggegeven en het einde)
```

### 2. doorlopen van boomstructuren

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

// De hele boom doorlopen
of(tree).pipe(
  expand(node =>
    node.children && node.children.length > 0
      ? from(node.children)
      : EMPTY
  )
).subscribe(node => {
  console.log(`ID: ${node.id}, Name: ${node.name}`);
});
// Uitvoer:
// ID: 1, Name: Root
// ID: 2, Name: Child 1
// ID: 3, Name: Child 2
// ID: 4, Name: Grandchild 1
// ID: 5, Name: Grandchild 2
// ID: 6, Name: Grandchild 3
```

### 3. paginering van de API

```ts
import { of, EMPTY } from 'rxjs';
import { expand, mergeMap } from 'rxjs';

interface PageResponse {
  data: string[];
  nextPage: number | null;
}

function fetchPage(page: number): Promise<PageResponse> {
  // APIVerzoek simuleren
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

// Alle pagina's opeenvolgend ophalen
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
  console.log(`Gegevens van de pagina:`, response.data);
});
```

#### Meer praktische uitvoering van paginering

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

// UIAanmaken van elementen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Voorbeeld implementatie paginering';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Alle gegevens ophalen';
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

// Gebruiksvoorbeeld：VoorbeeldAPIGebruikersgegevens ophalen met
interface User {
  id: number;
  name: string;
  email: string;
}

// VoorbeeldAPISimuleer een
async function fetchUsers(cursor: string | null): Promise<PaginatedResponse<User>> {
  // APISimuleer verzoeken (met100msVertraagd)
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

// Alle gegevens ophalen met één klik op een knop
button.addEventListener('click', async () => {
  button.disabled = true;
  status.textContent = 'Gegevensverwerving bezig...';
  output.textContent = '';

  try {
    const allUsers = await fetchPagedData(fetchUsers);

    status.textContent = `Acquisitie voltooid: ${allUsers.length}Gebruikersgegevens voor`;
    output.textContent = JSON.stringify(allUsers, null, 2);

    console.log(`Aantal gebruikers: ${allUsers.length}`);
    console.log('Gebruikersgegevens:', allUsers);
  } catch (error) {
    status.textContent = `Fout: ${error}`;
  } finally {
    button.disabled = false;
  }
});
```

## 🧠 Praktisch codevoorbeeld (weergave van directoryhiërarchie)

Dit is een voorbeeld van het recursief doorlopen van de mappenstructuur van een bestandssysteem.

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

// Structuur voorbeeldbestandssysteem
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

// UIAanmaken van elementen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Weergave mappenhiërarchie';
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

// Recursief uitgebreide mappenstructuur
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
    stats.textContent = `Aantal mappen: ${dirCount}, Aantal bestanden: ${fileCount}`;
  }
});
```

## 📋 Type veilig gebruik.

Dit is een voorbeeld van een typeveilige implementatie in TypeScript die gebruik maakt van generics.

```ts
import { Observable, of, from, EMPTY } from 'rxjs';
import { expand, filter, take, defaultIfEmpty, reduce } from 'rxjs';

interface Node<T> {
  value: T;
  children?: Node<T>[];
}

class TreeTraversal<T> {
  /**
   * Doorloopt de boomstructuur met zoeken eerst in de breedte
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
   * Zoek naar het eerste knooppunt dat aan de criteria voldoet
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
   * Telt het aantal knooppunten in de boomstructuur
   */
  countNodes(root: Node<T>): Observable<number> {
    return this.traverseBFS(root).pipe(
      reduce((count) => count + 1, 0)
    );
  }

  /**
   * Zoekt alle knooppunten met een specifieke waarde
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

// Gebruiksvoorbeeld
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

// De hele boom doorlopen
traversal.traverseBFS(tree).subscribe(node => {
  console.log(`Bezoekt: ${node.value}`);
});
// Uitvoer: Bezoekt: A, Bezoekt: B, Bezoekt: C, Bezoekt: D, Bezoekt: E, Bezoekt: F

// Zoekt naar een specifiek knooppunt
traversal.findNode(tree, value => value === 'D').subscribe(node => {
  console.log(`Gevonden knooppunten: ${node?.value}`);
});
// Uitvoer: Gevonden knooppunten: D

// Aantal knooppunten tellen
traversal.countNodes(tree).subscribe(count => {
  console.log(`Aantal knooppunten in de boom: ${count}`);
});
// Uitvoer: Aantal knooppunten in de boom: 6

// Krijg alle knooppunten die aan de criteria voldoen
traversal.findAllNodes(tree, value => value.length === 1).subscribe(nodes => {
  console.log(`Knooppunt met één teken: ${nodes.map(n => n.value).join(', ')}`);
});
// Uitvoer: Knooppunt met één teken: A, B, C, D, E, F
```

## 🎯 Gecombineerd met scheduler

`expand` werkt standaard synchroon, maar kan asynchroon worden bestuurd met een scheduler.

```ts
import { of, asyncScheduler } from 'rxjs';
import { expand, take } from 'rxjs';

// Synchroon (standaard)
console.log('Synchroon (standaard)expandBegint met');
of(1).pipe(
  expand(x => of(x * 2)),
  take(5)
).subscribe(x => console.log('Synchroon:', x));
console.log('Synchroon (standaard)expandEinde');
// Uitvoer:
// Synchroon (standaard)expandBegint met
// Synchroon: 1
// Synchroon: 2
// Synchroon: 4
// Synchroon: 8
// Synchroon: 16
// Synchroon (standaard)expandEinde

// Asynchroon (asyncSchedulerGebruiken)
console.log('AsynchroonexpandBegint met');
of(1, asyncScheduler).pipe(
  expand(x => of(x * 2, asyncScheduler)),
  take(5)
).subscribe(x => console.log('Asynchroon:', x));
console.log('AsynchroonexpandEinde');
// Uitvoer:
// AsynchroonexpandBegint met
// AsynchroonexpandEinde
// Asynchroon: 1
// Asynchroon: 2
// Asynchroon: 4
// Asynchroon: 8
// Asynchroon: 16
```

uitklappen_16___

> Bij het verwerken van grote hoeveelheden gegevens kan de `asyncScheduler` gebruikt worden om de UI responsief te houden zonder de hoofd thread te blokkeren. Voor meer informatie, zie [Scheduler types en hun gebruik](/nl/guide/schedulers/types).

## Voorbeeld van recursieve berekening

### Fibonacci-reeks.

```ts
import { of } from 'rxjs';
import { expand, take } from 'rxjs';

// 2Recursief proces van verdubbelen
of(1).pipe(
  expand(x => of(x * 2)),
  take(5) // Preventie van oneindige lus
).subscribe(console.log);
// Uitvoer: 1, 2, 4, 8, 16
```

### Factoriale berekeningen


## ⚠️ Veelgemaakte fouten


> De meest voorkomende fout met `uitbreiden` is om te vergeten een **exitvoorwaarde** in te stellen en in een oneindige lus** terecht te komen.

### Fout: geen afsluitvoorwaarde.


### Positief: met afsluitvoorwaarde


```ts
import { of } from 'rxjs';
import { expand, take } from 'rxjs';

// 2Recursief proces van verdubbelen
of(1).pipe(
  expand(x => of(x * 2)),
  take(5) // Preventie van oneindige lus
).subscribe(console.log);
// Uitvoer: 1, 2, 4, 8, 16
```


> Recursieve processen moeten altijd de exitconditie expliciet maken en oneindige lussen voorkomen door `take`, `takeWhile` of `EMPTY` terug te geven, afhankelijk van de conditie.

## Samenvatting

### Wanneer expand moet worden gebruikt.
- ✅ Wanneer je een boomstructuur of grafiek recursief wilt doorlopen.
- ✅ Als u alle gegevens met API-paginering wilt ophalen
- ✅ Als je recursieve berekeningen wilt uitvoeren (Fibonacci, factorial, etc.)
- ✅ Als je directorystructuren of bestandssystemen wilt doorkruisen
- ✅ Als je organigrammen of hiërarchische gegevens wilt onderzoeken

### Wanneer je mergeMap moet gebruiken.
- ✅ Als het voldoende is om elke waarde slechts één keer te converteren
- ✅ Normale asynchrone transformaties die geen recursieve verwerking vereisen

### Opmerkingen.
- ⚠️ **Stel altijd een afsluitvoorwaarde** (om oneindige lussen te voorkomen)
- ⚠️ Wees voorzichtig met geheugengebruik (bij het extraheren van grote hoeveelheden gegevens)
- ⚠️ Omdat het synchroon werkt, overweeg het gebruik van `asyncScheduler` voor grote hoeveelheden gegevens.
- ⚠️ Omdat debuggen moeilijk is, is het een goed idee om tussentijdse toestanden uit te loggen met `tap`.

## Volgende stappen.

- **[mergeMap](./mergeMap)** - leer de gebruikelijke asynchrone transformaties.
- **[switchMap](./switchMap)** - leer de conversie om over te schakelen naar het nieuwste proces.
- **[concatMap](./concatMap)** - leer transformaties die sequentieel worden uitgevoerd.
- **[Scheduler-types en hun gebruik](/nl/guide/schedulers/types)** - leer hoe u expand en schedulers kunt combineren.
- **[Praktische voorbeelden van conversie operatoren](/nl/guide/schedulers/types)** - leer hoe je expand en schedulers kunt combineren /praktische-gebruiksgevallen)** - leer over echte gebruiksgevallen
