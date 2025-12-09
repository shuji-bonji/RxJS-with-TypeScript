---
description: "Der expand-Operator ist ein RxJS-Operator, der aus jedem Wert ein neues Observable erzeugt und das Ergebnis rekursiv expandiert. Typsichere Implementierung rekursiver Datenerfassungsmuster wie Baumstruktur-Traversierung, API-Paginierung, unendliches Scrollen und Abhängigkeitsauflösung in TypeScript."
---

# expand - Rekursive Expansion

Der `expand`-Operator führt eine rekursive Transformation durch, die **aus jedem Wert ein neues Observable erzeugt und das Ergebnis ebenfalls expandiert**. Er ist ideal für Operationen, die Werte nacheinander expandieren, wie z.B. das Durchlaufen von Baumstrukturen, API-Paginierung und rekursive Berechnungen.


## 🔰 Grundlegende Syntax und Verwendung

```ts
import { of } from 'rxjs';
import { expand, take } from 'rxjs';

// Rekursive Verarbeitung: Verdoppeln
of(1).pipe(
  expand(x => of(x * 2)),
  take(5) // Endlosschleife verhindern
).subscribe(console.log);
// Ausgabe: 1, 2, 4, 8, 16
```

**Ablauf der Operation**:
1. Der Anfangswert `1` wird ausgegeben
2. Die `expand`-Funktion erhält `1` und gibt `of(2)` zurück
3. `2` wird ausgegeben und die `expand`-Funktion wird erneut aufgerufen
4. Die `expand`-Funktion erhält `2` und gibt `of(4)` zurück
5. Diese Iteration wiederholt sich...

> [!WARNING]
> `expand` führt zu einer **Endlosschleife**, wenn keine Abbruchbedingung angegeben wird. Stellen Sie sicher, dass Sie eine Abbruchbedingung wie `take` oder bedingte Rückgabe von `EMPTY` setzen.

[🌐 Offizielle RxJS-Dokumentation - `expand`](https://rxjs.dev/api/operators/expand)


## 🔄 Unterschied zu mergeMap

`expand` ist ähnlich wie `mergeMap`, verarbeitet aber auch **rekursiv die Ergebnisse des erzeugten Observable**.

```ts
import { of } from 'rxjs';
import { mergeMap, expand, take } from 'rxjs';

const double = (x: number) => of(x * 2);

// mergeMap: Nur einmalige Transformation
of(1).pipe(
  mergeMap(double),
  take(5)
).subscribe(console.log);
// Ausgabe: 2
// (Nur ein Wert, 2 wird nicht erneut transformiert)

// expand: Rekursive Transformation
of(1).pipe(
  expand(double),
  take(5)
).subscribe(console.log);
// Ausgabe: 1, 2, 4, 8, 16
// (Jedes Ergebnis wird erneut transformiert)
```

| Operator | Verarbeitung | Rekursiv | Anwendungsfall |
|---|---|---|---|
| `mergeMap` | Transformiert jeden Wert nur einmal | ❌ | Normale asynchrone Transformation |
| `expand` | Transformiert Ergebnisse rekursiv | ✅ | Baum-Traversierung, Paginierung, rekursive Berechnung |


## 💡 Typische Verwendungsmuster

### 1. Rekursive Verarbeitung mit Abbruchbedingung

```ts
import { of, EMPTY } from 'rxjs';
import { expand } from 'rxjs';

// Verdoppeln bis unter 10
of(1).pipe(
  expand(x => {
    const next = x * 2;
    return next < 10 ? of(next) : EMPTY;
  })
).subscribe(console.log);
// Ausgabe: 1, 2, 4, 8
// (16 ist >= 10, also wird EMPTY zurückgegeben und beendet)
```

### 2. Baumstruktur-Traversierung

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

// Gesamten Baum durchlaufen
of(tree).pipe(
  expand(node =>
    node.children && node.children.length > 0
      ? from(node.children)
      : EMPTY
  )
).subscribe(node => {
  console.log(`ID: ${node.id}, Name: ${node.name}`);
});
// Ausgabe:
// ID: 1, Name: Root
// ID: 2, Name: Child 1
// ID: 3, Name: Child 2
// ID: 4, Name: Grandchild 1
// ID: 5, Name: Grandchild 2
// ID: 6, Name: Grandchild 3
```

### 3. API-Paginierung

```ts
import { of, EMPTY } from 'rxjs';
import { expand, mergeMap } from 'rxjs';

interface PageResponse {
  data: string[];
  nextPage: number | null;
}

function fetchPage(page: number): Promise<PageResponse> {
  // API-Request simulieren
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

// Alle Seiten sequenziell abrufen
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
  console.log(`Seitendaten:`, response.data);
});
```

#### Praktischere Paginierungsimplementierung

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

// UI-Elemente erstellen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Paginierungsbeispiel';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Alle Daten abrufen';
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

// Verwendungsbeispiel: Benutzerdaten mit Mock-API abrufen
interface User {
  id: number;
  name: string;
  email: string;
}

// Mock-API simulieren
async function fetchUsers(cursor: string | null): Promise<PaginatedResponse<User>> {
  // API-Request simulieren (100ms Verzögerung)
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

// Button-Klick ruft alle Daten ab
button.addEventListener('click', async () => {
  button.disabled = true;
  status.textContent = 'Daten werden abgerufen...';
  output.textContent = '';

  try {
    const allUsers = await fetchPagedData(fetchUsers);

    status.textContent = `Abruf abgeschlossen: ${allUsers.length} Benutzerdatensätze`;
    output.textContent = JSON.stringify(allUsers, null, 2);

    console.log(`Gesamtanzahl Benutzer: ${allUsers.length}`);
    console.log('Benutzerdaten:', allUsers);
  } catch (error) {
    status.textContent = `Fehler: ${error}`;
  } finally {
    button.disabled = false;
  }
});
```


## 🧠 Praktisches Codebeispiel (Verzeichnishierarchie-Anzeige)

Ein Beispiel für das rekursive Durchlaufen einer Dateisystem-Verzeichnisstruktur.

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

// Beispiel-Dateisystemstruktur
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

// UI-Elemente erstellen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Verzeichnishierarchie-Anzeige';
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

// Verzeichnisstruktur rekursiv expandieren
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
    stats.textContent = `Verzeichnisse: ${dirCount}, Dateien: ${fileCount}`;
  }
});
```


## 📋 Typsichere Verwendung

Ein Beispiel für typsichere Implementierung mit TypeScript-Generics.

```ts
import { Observable, of, from, EMPTY } from 'rxjs';
import { expand, filter, take, defaultIfEmpty, reduce } from 'rxjs';

interface Node<T> {
  value: T;
  children?: Node<T>[];
}

class TreeTraversal<T> {
  /**
   * Baumstruktur mit Breitensuche durchlaufen
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
   * Ersten Knoten suchen, der der Bedingung entspricht
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
   * Alle Knoten im Baum zählen
   */
  countNodes(root: Node<T>): Observable<number> {
    return this.traverseBFS(root).pipe(
      reduce((count) => count + 1, 0)
    );
  }

  /**
   * Alle Knoten mit bestimmtem Wert abrufen
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

// Verwendungsbeispiel
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

// Gesamten Baum durchlaufen
traversal.traverseBFS(tree).subscribe(node => {
  console.log(`Besucht: ${node.value}`);
});
// Ausgabe: Besucht: A, Besucht: B, Besucht: C, Besucht: D, Besucht: E, Besucht: F

// Bestimmten Knoten suchen
traversal.findNode(tree, value => value === 'D').subscribe(node => {
  console.log(`Gefundener Knoten: ${node?.value}`);
});
// Ausgabe: Gefundener Knoten: D

// Knotenanzahl zählen
traversal.countNodes(tree).subscribe(count => {
  console.log(`Knotenanzahl im Baum: ${count}`);
});
// Ausgabe: Knotenanzahl im Baum: 6

// Alle Knoten abrufen, die der Bedingung entsprechen
traversal.findAllNodes(tree, value => value.length === 1).subscribe(nodes => {
  console.log(`Einzelzeichen-Knoten: ${nodes.map(n => n.value).join(', ')}`);
});
// Ausgabe: Einzelzeichen-Knoten: A, B, C, D, E, F
```


## 🎯 Kombination mit Scheduler

`expand` arbeitet standardmäßig synchron, kann aber mit einem Scheduler asynchron gesteuert werden.

```ts
import { of, asyncScheduler } from 'rxjs';
import { expand, take } from 'rxjs';

// Synchron (Standard)
console.log('Synchrones expand Start');
of(1).pipe(
  expand(x => of(x * 2)),
  take(5)
).subscribe(x => console.log('Synchron:', x));
console.log('Synchrones expand Ende');
// Ausgabe:
// Synchrones expand Start
// Synchron: 1
// Synchron: 2
// Synchron: 4
// Synchron: 8
// Synchron: 16
// Synchrones expand Ende

// Asynchron (mit asyncScheduler)
console.log('Asynchrones expand Start');
of(1, asyncScheduler).pipe(
  expand(x => of(x * 2, asyncScheduler)),
  take(5)
).subscribe(x => console.log('Asynchron:', x));
console.log('Asynchrones expand Ende');
// Ausgabe:
// Asynchrones expand Start
// Asynchrones expand Ende
// Asynchron: 1
// Asynchron: 2
// Asynchron: 4
// Asynchron: 8
// Asynchron: 16
```

> [!TIP]
> Bei der Verarbeitung großer Datenmengen kann der `asyncScheduler` verwendet werden, um die UI reaktionsfähig zu halten, ohne den Hauptthread zu blockieren. Weitere Informationen finden Sie unter [Scheduler-Typen und ihre Verwendung](/de/guide/schedulers/types).


## 🔄 Beispiele für rekursive Berechnungen

### Fibonacci-Folge

```ts
import { of, EMPTY } from 'rxjs';
import { expand, map, take } from 'rxjs';

interface FibState {
  current: number;
  next: number;
}

// Fibonacci-Folge generieren
of({ current: 0, next: 1 } as FibState).pipe(
  expand(state =>
    state.current < 100
      ? of({ current: state.next, next: state.current + state.next })
      : EMPTY
  ),
  map(state => state.current),
  take(10)
).subscribe(n => console.log(n));
// Ausgabe: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

### Fakultätsberechnung

```ts
import { of, EMPTY, Observable } from 'rxjs';
import { expand, reduce } from 'rxjs';

interface FactorialState {
  n: number;
  result: number;
}

function factorial(n: number): Observable<number> {
  return of({ n, result: 1 } as FactorialState).pipe(
    expand(state =>
      state.n > 1
        ? of({ n: state.n - 1, result: state.result * state.n })
        : EMPTY
    ),
    reduce((acc, state) => state.result, 1)
  );
}

factorial(5).subscribe(result => {
  console.log('5! =', result); // 5! = 120
});
```


## ⚠️ Häufige Fehler

> [!WARNING]
> Der häufigste Fehler bei `expand` ist das **Vergessen einer Abbruchbedingung, was zu einer Endlosschleife führt**.

### Falsch: Keine Abbruchbedingung

```ts
import { of } from 'rxjs';
import { expand } from 'rxjs';

// ❌ Schlechtes Beispiel: Endlosschleife
of(1).pipe(
  expand(x => of(x + 1))
).subscribe(console.log);
// Verursacht Speicherleck und Browser-Einfrieren
```

### Richtig: Mit Abbruchbedingung

```ts
import { of, EMPTY } from 'rxjs';
import { expand, take, takeWhile } from 'rxjs';

// ✅ Gutes Beispiel 1: Begrenzung mit take
of(1).pipe(
  expand(x => of(x + 1)),
  take(10)
).subscribe(console.log);

// ✅ Gutes Beispiel 2: Bedingtes EMPTY zurückgeben
of(1).pipe(
  expand(x => x < 10 ? of(x + 1) : EMPTY)
).subscribe(console.log);

// ✅ Gutes Beispiel 3: Begrenzung mit takeWhile
of(1).pipe(
  expand(x => of(x + 1)),
  takeWhile(x => x <= 10)
).subscribe(console.log);
```

> [!IMPORTANT]
> Bei rekursiver Verarbeitung sollten Sie immer die Abbruchbedingung explizit festlegen und Endlosschleifen mit `take`, `takeWhile` oder bedingter Rückgabe von `EMPTY` verhindern.


## 🎓 Zusammenfassung

### Wann expand verwenden
- ✅ Wenn Sie Baumstrukturen oder Graphen rekursiv durchlaufen möchten
- ✅ Wenn Sie alle Daten mit API-Paginierung abrufen möchten
- ✅ Wenn Sie rekursive Berechnungen durchführen möchten (Fibonacci, Fakultät usw.)
- ✅ Wenn Sie Verzeichnisstrukturen oder Dateisysteme durchlaufen möchten
- ✅ Wenn Sie Organigramme oder hierarchische Daten erkunden möchten

### Wann mergeMap verwenden
- ✅ Wenn einmalige Transformation jedes Werts ausreicht
- ✅ Bei normalen asynchronen Transformationen, die keine rekursive Verarbeitung erfordern

### Hinweise
- ⚠️ **Immer Abbruchbedingung setzen** (Endlosschleifen vermeiden)
- ⚠️ Auf Speicherverbrauch achten (bei großen Datenmengen)
- ⚠️ Da synchron arbeitend, bei großen Datenmengen `asyncScheduler` in Betracht ziehen
- ⚠️ Da Debugging schwierig ist, Zwischenzustände mit `tap` protokollieren


## 🚀 Nächste Schritte

- **[mergeMap](./mergeMap)** - Normale asynchrone Transformation lernen
- **[switchMap](./switchMap)** - Zur neuesten Verarbeitung wechselnde Transformation lernen
- **[concatMap](./concatMap)** - Sequenzielle Transformation lernen
- **[Scheduler-Typen und ihre Verwendung](/de/guide/schedulers/types)** - expand mit Scheduler kombinieren lernen
- **[Praktische Beispiele für Transformationsoperatoren](./practical-use-cases)** - Reale Anwendungsfälle lernen
