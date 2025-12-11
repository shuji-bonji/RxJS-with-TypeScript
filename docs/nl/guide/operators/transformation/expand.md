---
description: De expand operator is een RxJS operator die een nieuwe Observable maakt van elke waarde en het resultaat recursief uitbreidt. Het kan worden gebruikt voor boomstructuur-traversal, API-paginering, recursieve berekening, en meer.
---

# expand - Recursieve uitbreiding

De `expand` operator voert een recursieve transformatie uit die **een nieuwe Observable genereert van elke waarde en het resultaat ook uitbreidt**. Het is het beste geschikt voor operaties die waarden één voor één uitbreiden, zoals het doorlopen van een boomstructuur, API-paginering, of recursieve berekening.

## 🔰 Basissyntax en gebruik

```ts
import { of } from 'rxjs';
import { expand, take } from 'rxjs';

// Recursieve verwerking die verdubbelt
of(1).pipe(
  expand(x => of(x * 2)),
  take(5) // Voorkom oneindige lus
).subscribe(console.log);
// Output: 1, 2, 4, 8, 16
```

**Werkingsstroom**:
1. Initiële waarde `1` wordt uitgegeven
2. Functie `expand` ontvangt `1` en retourneert `of(2)`
3. `2` wordt uitgegeven en de functie `expand` wordt opnieuw aangeroepen
4. Functie `expand` ontvangt `2` en retourneert `of(4)`
5. Deze iteratie gaat door...

> [!WARNING]
> `expand` zal **ONEINDIG LUSSEN** als u geen exitvoorwaarde specificeert. Zorg ervoor dat u een exitvoorwaarde instelt zoals `take` of conditioneel `EMPTY` retourneert.

[🌐 RxJS Officiële Documentatie - `expand`](https://rxjs.dev/api/operators/expand)

## 🔄 Verschil met mergeMap

`expand` lijkt op `mergeMap`, behalve dat het ook **recursief de resultaten verwerkt** van de gegenereerde Observable.

```ts
import { of } from 'rxjs';
import { mergeMap, expand, take } from 'rxjs';

const double = (x: number) => of(x * 2);

// mergeMap: Transformeer slechts één keer
of(1).pipe(
  mergeMap(double),
  take(5)
).subscribe(console.log);
// Output: 2
// (Slechts één waarde, 2 wordt niet opnieuw getransformeerd)

// expand: Recursieve transformatie
of(1).pipe(
  expand(double),
  take(5)
).subscribe(console.log);
// Output: 1, 2, 4, 8, 16
// (Elk resultaat wordt opnieuw getransformeerd)
```

| Operator | Verwerking | Recursief | Gebruiksscenario |
|---|---|---|---|
| `mergeMap` | Transformeer elke waarde slechts één keer | ❌ | Normale asynchrone transformatie |
| `expand` | Transformeer het resultaat recursief | ✅ | Boomtraversal, paginering, recursieve berekening |

## 💡 Typische gebruikspatronen

### 1. Recursieve verwerking met beëindigingsvoorwaarden

```ts
import { of, EMPTY } from 'rxjs';
import { expand } from 'rxjs';

// Verdubbel tot minder dan 10
of(1).pipe(
  expand(x => {
    const next = x * 2;
    return next < 10 ? of(next) : EMPTY;
  })
).subscribe(console.log);
// Output: 1, 2, 4, 8
// (16 is >= 10, dus EMPTY wordt geretourneerd en het eindigt)
```

### 2. Boomstructuur traversal

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
      name: 'Kind 1',
      children: [
        { id: 4, name: 'Kleinkind 1' },
        { id: 5, name: 'Kleinkind 2' }
      ]
    },
    {
      id: 3,
      name: 'Kind 2',
      children: [
        { id: 6, name: 'Kleinkind 3' }
      ]
    }
  ]
};

// Doorloop de hele boom
of(tree).pipe(
  expand(node =>
    node.children && node.children.length > 0
      ? from(node.children)
      : EMPTY
  )
).subscribe(node => {
  console.log(`ID: ${node.id}, Naam: ${node.name}`);
});
// Output:
// ID: 1, Naam: Root
// ID: 2, Naam: Kind 1
// ID: 3, Naam: Kind 2
// ID: 4, Naam: Kleinkind 1
// ID: 5, Naam: Kleinkind 2
// ID: 6, Naam: Kleinkind 3
```

### 3. API-paginering

```ts
import { of, EMPTY } from 'rxjs';
import { expand, mergeMap } from 'rxjs';

interface PageResponse {
  data: string[];
  nextPage: number | null;
}

function fetchPage(page: number): Promise<PageResponse> {
  // Simuleer API-verzoek
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

// Haal alle pagina's sequentieel op
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
  console.log(`Paginadata:`, response.data);
});
```

## 🧠 Praktisch codevoorbeeld (Toon directoryhiërarchie)

Dit is een voorbeeld van het recursief doorlopen van de directorystructuur van een bestandssysteem.

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

// Voorbeeld bestandssysteemstructuur
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

// Maak UI-elementen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Directoryhiërarchie weergave';
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

// Breid directorystructuur recursief uit
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
    stats.textContent = `Directories: ${dirCount}, Bestanden: ${fileCount}`;
  }
});
```

## ⚠️ Veelgemaakte fouten

> [!WARNING]
> De meest voorkomende fout met `expand` is **het vergeten een exitvoorwaarde in te stellen, wat resulteert in een oneindige lus**.

### Fout: Geen exitvoorwaarde

```ts
import { of } from 'rxjs';
import { expand } from 'rxjs';

// ❌ Slecht voorbeeld: Oneindige lus
of(1).pipe(
  expand(x => of(x + 1))
).subscribe(console.log);
// Veroorzaakt geheugenlek en browser bevriest
```

### Correct: Met exitvoorwaarde

```ts
import { of, EMPTY } from 'rxjs';
import { expand, take, takeWhile } from 'rxjs';

// ✅ Goed voorbeeld 1: Beperk aantal met take
of(1).pipe(
  expand(x => of(x + 1)),
  take(10)
).subscribe(console.log);

// ✅ Goed voorbeeld 2: Retourneer EMPTY conditioneel
of(1).pipe(
  expand(x => x < 10 ? of(x + 1) : EMPTY)
).subscribe(console.log);

// ✅ Goed voorbeeld 3: Voorwaarde limiet met takeWhile
of(1).pipe(
  expand(x => of(x + 1)),
  takeWhile(x => x <= 10)
).subscribe(console.log);
```

> [!IMPORTANT]
> Maak bij recursieve verwerking altijd de exitvoorwaarde expliciet en voorkom oneindige lussen door `take`, `takeWhile`, of `EMPTY` te retourneren afhankelijk van de voorwaarde.

## 🎓 Samenvatting

### Wanneer zou expand moeten worden gebruikt?
- ✅ Als u een boomstructuur of graaf recursief wilt doorlopen
- ✅ Wanneer u alle data in API-paginering wilt ophalen
- ✅ Als u recursieve berekeningen wilt uitvoeren (Fibonacci, faculteit, etc.)
- ✅ Als u een directorystructuur of bestandssysteem wilt doorlopen
- ✅ Om organisatieschema's en hiërarchische data te verkennen

### Wanneer zou u mergeMap moeten gebruiken?
- ✅ Wanneer het voldoende is om elke waarde slechts één keer te converteren
- ✅ Normale asynchrone conversies die geen recursieve verwerking vereisen

### Let op
- ⚠️ **Stel altijd een exitvoorwaarde in** (om oneindige lussen te voorkomen)
- ⚠️ Wees voorzichtig met geheugenverbruik (bij het extraheren van grote hoeveelheden data)
- ⚠️ Omdat het synchroon werkt, overweeg `asyncScheduler` te gebruiken voor grote hoeveelheden data
- ⚠️ Omdat debuggen moeilijk is, is het goed om `tap` te gebruiken om tussenstatussen te loggen

## 🚀 Volgende stappen

- **[mergeMap](/nl/guide/operators/transformation/mergeMap)** - Leer normale asynchrone conversie
- **[switchMap](/nl/guide/operators/transformation/switchMap)** - Leer de conversie om naar het nieuwste proces te schakelen
- **[concatMap](/nl/guide/operators/transformation/concatMap)** - Leer conversies die sequentieel worden uitgevoerd
- **[Scheduler types en gebruik](/nl/guide/schedulers/types)** - Leer expand en schedulers combineren
- **[Transformatieoperator praktische voorbeelden](/nl/guide/operators/transformation/practical-use-cases)** - Leer echte use cases
