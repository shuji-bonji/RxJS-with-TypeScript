---
description: "L'opérateur expand génère un nouvel Observable à partir de chaque valeur et développe récursivement ses résultats. Il implémente de manière type-safe en TypeScript des patterns de récupération récursive de données tels que le parcours de structures arborescentes, la pagination d'API, le défilement infini et la résolution de dépendances."
---

# expand - Expansion récursive

L'opérateur `expand` effectue une transformation récursive qui **génère un nouvel Observable à partir de chaque valeur et développe également le résultat**. Il est idéal pour les opérations qui développent les valeurs les unes après les autres, comme le parcours de structures arborescentes, la pagination d'API et les calculs récursifs.


## 🔰 Syntaxe de base et utilisation

```ts
import { of } from 'rxjs';
import { expand, take } from 'rxjs';

// Traitement récursif qui double la valeur
of(1).pipe(
  expand(x => of(x * 2)),
  take(5) // Prévention de boucle infinie
).subscribe(console.log);
// Sortie : 1, 2, 4, 8, 16
```

**Flux d'opérations** :
1. La valeur initiale `1` est émise
2. La fonction `expand` reçoit `1` et renvoie `of(2)`
3. `2` est émis et la fonction `expand` est appelée à nouveau
4. La fonction `expand` reçoit `2` et renvoie `of(4)`
5. Ce cycle se répète...

> [!WARNING]
> `expand` résultera en une **boucle infinie** si aucune condition de sortie n'est spécifiée. Assurez-vous de définir une condition de sortie, comme `take` ou retourner conditionnellement `EMPTY`.

[🌐 Documentation officielle RxJS - `expand`](https://rxjs.dev/api/operators/expand)


## 🔄 Différences avec mergeMap

`expand` est similaire à `mergeMap`, sauf qu'il **traite récursivement les résultats de l'Observable généré**.

```ts
import { of } from 'rxjs';
import { mergeMap, expand, take } from 'rxjs';

const double = (x: number) => of(x * 2);

// mergeMap : transformation unique
of(1).pipe(
  mergeMap(double),
  take(5)
).subscribe(console.log);
// Sortie : 2
// (une seule valeur, 2 n'est pas transformé à nouveau)

// expand : transformation récursive
of(1).pipe(
  expand(double),
  take(5)
).subscribe(console.log);
// Sortie : 1, 2, 4, 8, 16
// (chaque résultat est transformé à nouveau)
```

| Opérateur | Traitement | Récursif | Cas d'utilisation |
|---|---|---|---|
| `mergeMap` | Transforme chaque valeur une seule fois | ❌ | Transformations asynchrones normales |
| `expand` | Transforme les résultats récursivement | ✅ | Parcours d'arbres, pagination, calculs récursifs |


## 💡 Patterns d'utilisation typiques

### 1. Traitement récursif avec condition de sortie

```ts
import { of, EMPTY } from 'rxjs';
import { expand } from 'rxjs';

// Doubler jusqu'à moins de 10
of(1).pipe(
  expand(x => {
    const next = x * 2;
    return next < 10 ? of(next) : EMPTY;
  })
).subscribe(console.log);
// Sortie : 1, 2, 4, 8
// (16 est >= 10 donc EMPTY est retourné, fin)
```

### 2. Parcours de structure arborescente

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

// Parcourir tout l'arbre
of(tree).pipe(
  expand(node =>
    node.children && node.children.length > 0
      ? from(node.children)
      : EMPTY
  )
).subscribe(node => {
  console.log(`ID: ${node.id}, Name: ${node.name}`);
});
// Sortie :
// ID: 1, Name: Root
// ID: 2, Name: Child 1
// ID: 3, Name: Child 2
// ID: 4, Name: Grandchild 1
// ID: 5, Name: Grandchild 2
// ID: 6, Name: Grandchild 3
```

### 3. Pagination d'API

```ts
import { of, EMPTY } from 'rxjs';
import { expand, mergeMap } from 'rxjs';

interface PageResponse {
  data: string[];
  nextPage: number | null;
}

function fetchPage(page: number): Promise<PageResponse> {
  // Simulation de requête API
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

// Récupérer toutes les pages séquentiellement
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
  console.log(`Données de la page :`, response.data);
});
```

#### Implémentation plus pratique de la pagination

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

// Création des éléments UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Exemple d\'implémentation de pagination';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Récupérer toutes les données';
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

// Exemple d'utilisation : récupérer les données utilisateurs avec une API simulée
interface User {
  id: number;
  name: string;
  email: string;
}

// Simulation d'API
async function fetchUsers(cursor: string | null): Promise<PaginatedResponse<User>> {
  // Simulation de requête API (délai de 100ms)
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

// Récupérer toutes les données au clic du bouton
button.addEventListener('click', async () => {
  button.disabled = true;
  status.textContent = 'Récupération des données...';
  output.textContent = '';

  try {
    const allUsers = await fetchPagedData(fetchUsers);

    status.textContent = `Terminé : ${allUsers.length} données utilisateurs`;
    output.textContent = JSON.stringify(allUsers, null, 2);

    console.log(`Nombre total d'utilisateurs : ${allUsers.length}`);
    console.log('Données utilisateurs :', allUsers);
  } catch (error) {
    status.textContent = `Erreur : ${error}`;
  } finally {
    button.disabled = false;
  }
});
```


## 🧠 Exemple de code pratique (affichage de la hiérarchie des répertoires)

Voici un exemple de parcours récursif de la structure des répertoires d'un système de fichiers.

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

// Exemple de structure de système de fichiers
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

// Création des éléments UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Affichage de la hiérarchie des répertoires';
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

// Développer récursivement la structure des répertoires
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
    stats.textContent = `Répertoires : ${dirCount}, Fichiers : ${fileCount}`;
  }
});
```


## 📋 Utilisation type-safe

Un exemple d'implémentation type-safe utilisant les génériques en TypeScript.

```ts
import { Observable, of, from, EMPTY } from 'rxjs';
import { expand, filter, take, defaultIfEmpty, reduce } from 'rxjs';

interface Node<T> {
  value: T;
  children?: Node<T>[];
}

class TreeTraversal<T> {
  /**
   * Parcourir la structure arborescente en largeur d'abord (BFS)
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
   * Rechercher le premier nœud correspondant à la condition
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
   * Compter tous les nœuds de l'arbre
   */
  countNodes(root: Node<T>): Observable<number> {
    return this.traverseBFS(root).pipe(
      reduce((count) => count + 1, 0)
    );
  }

  /**
   * Obtenir tous les nœuds avec une valeur spécifique
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

// Exemple d'utilisation
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

// Parcourir tout l'arbre
traversal.traverseBFS(tree).subscribe(node => {
  console.log(`Visite : ${node.value}`);
});
// Sortie : Visite : A, Visite : B, Visite : C, Visite : D, Visite : E, Visite : F

// Rechercher un nœud spécifique
traversal.findNode(tree, value => value === 'D').subscribe(node => {
  console.log(`Nœud trouvé : ${node?.value}`);
});
// Sortie : Nœud trouvé : D

// Compter les nœuds
traversal.countNodes(tree).subscribe(count => {
  console.log(`Nombre de nœuds dans l'arbre : ${count}`);
});
// Sortie : Nombre de nœuds dans l'arbre : 6

// Obtenir tous les nœuds correspondants
traversal.findAllNodes(tree, value => value.length === 1).subscribe(nodes => {
  console.log(`Nœuds à un seul caractère : ${nodes.map(n => n.value).join(', ')}`);
});
// Sortie : Nœuds à un seul caractère : A, B, C, D, E, F
```


## 🎯 Combinaison avec les Schedulers

`expand` fonctionne de manière synchrone par défaut, mais peut être contrôlé de manière asynchrone à l'aide d'un scheduler.

```ts
import { of, asyncScheduler } from 'rxjs';
import { expand, take } from 'rxjs';

// Synchrone (par défaut)
console.log('Début expand synchrone');
of(1).pipe(
  expand(x => of(x * 2)),
  take(5)
).subscribe(x => console.log('Sync :', x));
console.log('Fin expand synchrone');
// Sortie :
// Début expand synchrone
// Sync : 1
// Sync : 2
// Sync : 4
// Sync : 8
// Sync : 16
// Fin expand synchrone

// Asynchrone (avec asyncScheduler)
console.log('Début expand asynchrone');
of(1, asyncScheduler).pipe(
  expand(x => of(x * 2, asyncScheduler)),
  take(5)
).subscribe(x => console.log('Async :', x));
console.log('Fin expand asynchrone');
// Sortie :
// Début expand asynchrone
// Fin expand asynchrone
// Async : 1
// Async : 2
// Async : 4
// Async : 8
// Async : 16
```

> [!TIP]
> Lorsque vous traitez de grandes quantités de données, vous pouvez utiliser `asyncScheduler` pour garder l'interface utilisateur réactive sans bloquer le thread principal. Pour plus d'informations, voir [Types de Schedulers et leur utilisation](/fr/guide/schedulers/types).


## 🔄 Exemples de calculs récursifs

### Séquence de Fibonacci

```ts
import { of, EMPTY } from 'rxjs';
import { expand, map, take } from 'rxjs';

interface FibState {
  current: number;
  next: number;
}

// Générer la séquence de Fibonacci
of({ current: 0, next: 1 } as FibState).pipe(
  expand(state =>
    state.current < 100
      ? of({ current: state.next, next: state.current + state.next })
      : EMPTY
  ),
  map(state => state.current),
  take(10)
).subscribe(n => console.log(n));
// Sortie : 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

### Calcul de factorielle

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


## ⚠️ Erreurs courantes

> [!WARNING]
> L'erreur la plus courante avec `expand` est **d'oublier de définir une condition de sortie, ce qui entraîne une boucle infinie**.

### Incorrect : pas de condition de sortie

```ts
import { of } from 'rxjs';
import { expand } from 'rxjs';

// ❌ Mauvais exemple : boucle infinie
of(1).pipe(
  expand(x => of(x + 1))
).subscribe(console.log);
// Provoque une fuite de mémoire et un gel du navigateur
```

### Correct : avec condition de sortie

```ts
import { of, EMPTY } from 'rxjs';
import { expand, take, takeWhile } from 'rxjs';

// ✅ Bon exemple 1 : limite par nombre avec take
of(1).pipe(
  expand(x => of(x + 1)),
  take(10)
).subscribe(console.log);

// ✅ Bon exemple 2 : retourner EMPTY conditionnellement
of(1).pipe(
  expand(x => x < 10 ? of(x + 1) : EMPTY)
).subscribe(console.log);

// ✅ Bon exemple 3 : limite par condition avec takeWhile
of(1).pipe(
  expand(x => of(x + 1)),
  takeWhile(x => x <= 10)
).subscribe(console.log);
```

> [!IMPORTANT]
> Dans les traitements récursifs, rendez toujours la condition de sortie explicite et évitez les boucles infinies en utilisant `take`, `takeWhile`, ou en retournant `EMPTY` selon la condition.


## 🎓 Résumé

### Quand utiliser expand
- ✅ Lorsque vous souhaitez parcourir récursivement une structure arborescente ou un graphe
- ✅ Lorsque vous voulez obtenir toutes les données avec la pagination d'API
- ✅ Lorsque vous souhaitez effectuer des calculs récursifs (Fibonacci, factorielle, etc.)
- ✅ Lorsque vous souhaitez parcourir des structures de répertoires ou des systèmes de fichiers
- ✅ Lorsque vous souhaitez explorer des organigrammes ou des données hiérarchiques

### Quand utiliser mergeMap
- ✅ Lorsqu'il suffit de convertir chaque valeur une seule fois
- ✅ Transformations asynchrones normales qui ne nécessitent pas de traitement récursif

### Points d'attention
- ⚠️ **Toujours définir une condition de sortie** (pour éviter les boucles infinies)
- ⚠️ Faites attention à la consommation de mémoire (lors du développement de grandes quantités de données)
- ⚠️ Parce qu'il fonctionne de manière synchrone, pensez à utiliser `asyncScheduler` pour de grandes quantités de données
- ⚠️ Parce que le débogage est difficile, la journalisation des états intermédiaires avec `tap` est une bonne idée


## 🚀 Prochaines étapes

- **[mergeMap](./mergeMap)** - Apprendre les transformations asynchrones habituelles
- **[switchMap](./switchMap)** - Apprendre les transformations qui passent au processus le plus récent
- **[concatMap](./concatMap)** - Apprendre les transformations exécutées séquentiellement
- **[Types de Schedulers et leur utilisation](/fr/guide/schedulers/types)** - Apprendre la combinaison expand et scheduler
- **[Exemples pratiques d'opérateurs de transformation](./practical-use-cases)** - Apprendre les cas d'utilisation réels
