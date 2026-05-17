---
description: "Explication détaillée de 15 erreurs courantes lors de l'utilisation de RxJS avec TypeScript, avec des exemples de code concrets. Guide pratique pour prévenir les problèmes fréquents sur le terrain : exposition externe des Subjects, souscriptions imbriquées, oubli de désinscription, gestion d'erreurs inappropriée."
---

# Erreurs courantes et solutions

Cette page explique en détail 15 anti-patterns fréquemment rencontrés lors de l'utilisation de RxJS avec TypeScript, et leurs solutions respectives.

## Table des matières

1. [Exposition externe des Subjects](#1-subject-の外部公開)
2. [Souscriptions imbriquées (callback hell)](#2-ネストした-subscribe-コールバック地獄)
3. [Oubli d'unsubscribe (fuites mémoire)](#3-unsubscribe-忘れ-メモリリーク)
4. [Mauvaise utilisation de shareReplay](#4-sharereplay-の誤用)
5. [Effets de bord dans map](#5-map-での副作用)
6. [Ignorer les différences Cold/Hot Observable](#6-cold-hot-observable-の違いの無視)
7. [Mélange inapproprié de Promise et Observable](#7-promise-と-observable-の不適切な混在)
8. [Ignorer le backpressure](#8-バックプレッシャーの無視)
9. [Suppression d'erreurs](#9-エラーの握りつぶし)
10. [Fuites de souscriptions d'événements DOM](#10-dom-イベントサブスクリプションのリーク)
11. [Manque de sécurité de type (utilisation excessive de any)](#11-型安全性の欠如-any-の多用)
12. [Sélection d'opérateur inappropriée](#12-不適切なオペレーター選択)
13. [Complexification excessive](#13-過度な複雑化)
14. [Modification d'état dans subscribe](#14-subscribe-内での状態変更)
15. [Absence de tests](#15-テストの欠如)


## 1. Exposition externe des Subjects

### Problème

Exposer directement un `Subject` permet aux externes d'appeler `next()`, rendant la gestion d'état imprévisible.

### ❌ Mauvais exemple

```ts
import { Subject } from 'rxjs';

// Export direct du Subject
export const cartChanged$ = new Subject<void>();

// N'importe qui peut appeler next() depuis un autre fichier
cartChanged$.next(); // Peut être appelé à des moments inattendus
```

### ✅ Bon exemple

```ts
import { BehaviorSubject, Observable } from 'rxjs';

class CartStore {
  private readonly _items$ = new BehaviorSubject<string[]>([]);

  // Exposer comme Observable en lecture seule
  readonly items$: Observable<string[]> = this._items$.asObservable();

  // Contrôler les changements d'état via des méthodes dédiées
  add(item: string): void {
    this._items$.next([...this._items$.value, item]);
  }

  remove(item: string): void {
    this._items$.next(
      this._items$.value.filter(i => i !== item)
    );
  }
}

export const cartStore = new CartStore();
```

### Explication

- Convertir en `Observable` en lecture seule avec `asObservable()`
- Permettre les changements d'état uniquement via des méthodes dédiées
- Améliorer la traçabilité des changements et faciliter le débogage


## 2. Souscriptions imbriquées (callback hell)

### Problème

Appeler `subscribe` à l'intérieur d'un autre `subscribe` conduit au callback hell, compliquant la gestion d'erreurs et l'annulation.

### ❌ Mauvais exemple

```ts
import { of } from 'rxjs';

// Simulation d'appels API
function apiA() {
  return of({ id: 1 });
}

function apiB(id: number) {
  return of({ id, token: 'abc123' });
}

function apiC(token: string) {
  return of({ success: true });
}

// Souscriptions imbriquées
apiA().subscribe(a => {
  apiB(a.id).subscribe(b => {
    apiC(b.token).subscribe(result => {
      console.log('done', result);
    });
  });
});
```

### ✅ Bon exemple

```ts
import { of } from 'rxjs';
import { switchMap } from 'rxjs';

function apiA() {
  return of({ id: 1 });
}

function apiB(id: number) {
  return of({ id, token: 'abc123' });
}

function apiC(token: string) {
  return of({ success: true });
};


// Aplatir avec des opérateurs d'ordre supérieur
apiA().pipe(
  switchMap(a => apiB(a.id)),
  switchMap(b => apiC(b.token))
).subscribe(result => {
  console.log('done', result);
});
```

### Explication

- Utiliser les opérateurs d'ordre supérieur comme `switchMap`, `mergeMap`, `concatMap`
- Gestion d'erreurs possible en un seul endroit
- Désinscription en une seule fois
- Lisibilité du code améliorée


## 3. Oubli d'unsubscribe (fuites mémoire)

### Problème

Ne pas se désabonner des flux infinis (comme les écouteurs d'événements) provoque des fuites mémoire.

### ❌ Mauvais exemple

```ts
import { fromEvent } from 'rxjs';

// Lors de l'initialisation du composant
function setupResizeHandler() {
  fromEvent(window, 'resize').subscribe(() => {
    console.log('resized');
  });
  // Pas de désinscription !
}

// L'écouteur d'événements persiste même après la destruction du composant
```

### ✅ Bon exemple

```ts
import { fromEvent, Subject } from 'rxjs';
import { takeUntil, finalize } from 'rxjs';

class MyComponent {
  private readonly destroy$ = new Subject<void>();

  ngOnInit(): void {
    fromEvent(window, 'resize').pipe(
      takeUntil(this.destroy$),
      finalize(() => console.log('cleanup'))
    ).subscribe(() => {
      console.log('resized');
    });
  }

  ngOnDestroy(): void {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

### ✅ Autre bon exemple (utilisant Subscription)

```ts
import { fromEvent, Subscription } from 'rxjs';

class MyComponent {
  private subscription = new Subscription();

  ngOnInit(): void {
    this.subscription.add(
      fromEvent(window, 'resize').subscribe(() => {
        console.log('resized');
      })
    );
  }

  ngOnDestroy(): void {
    this.subscription.unsubscribe();
  }
}
```

### Explication

- Le pattern `takeUntil` est recommandé (déclaratif et clair)
- La gestion manuelle avec `Subscription` est également efficace
- Toujours exécuter la désinscription lors de la destruction du composant


## 4. Mauvaise utilisation de shareReplay

### Problème

Utiliser `shareReplay` sans comprendre son fonctionnement peut entraîner la relecture de données périmées ou des fuites mémoire.

### ❌ Mauvais exemple

```ts
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

// Taille de buffer illimitée
const shared$ = interval(1000).pipe(
  shareReplay() // Buffer illimité par défaut
);

// Les valeurs restent en mémoire même sans abonnés
```

### ✅ Bon exemple

```ts
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

// Spécifier explicitement la taille du buffer et le comptage de références
const shared$ = interval(1000).pipe(
  take(10),
  shareReplay({
    bufferSize: 1,
    refCount: true // Libérer les ressources quand il n'y a plus d'abonnés
  })
);
```

### Explication

- Spécifier explicitement `bufferSize` (généralement 1)
- `refCount: true` pour libérer automatiquement quand il n'y a plus d'abonnés
- Pour les flux qui se terminent comme les requêtes HTTP, `shareReplay({ bufferSize: 1, refCount: true })` est sûr


## 5. Effets de bord dans map

### Problème

Modifier l'état dans l'opérateur `map` provoque un comportement imprévisible.

### ❌ Mauvais exemple

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

let counter = 0;

const source$ = of(1, 2, 3).pipe(
  map(value => {
    counter++; // Effet de bord !
    return value * 2;
  })
);

source$.subscribe(console.log);
source$.subscribe(console.log); // counter augmente de manière inattendue
```

### ✅ Bon exemple

```ts
import { of } from 'rxjs';
import { map, tap, scan } from 'rxjs';

// Transformation pure uniquement
const source$ = of(1, 2, 3).pipe(
  map(value => value * 2)
);

// Séparer les effets de bord avec tap
const withLogging$ = source$.pipe(
  tap(value => console.log('Processing:', value))
);

// Utiliser scan pour l'accumulation d'état
const withCounter$ = of(1, 2, 3).pipe(
  scan((acc, value) => ({ count: acc.count + 1, value }), { count: 0, value: 0 })
);
```

### Explication

- Utiliser `map` comme fonction pure
- Séparer les effets de bord (logs, appels API, etc.) avec `tap`
- Utiliser `scan` ou `reduce` pour l'accumulation d'état


## 6. Ignorer les différences Cold/Hot Observable

### Problème

Utiliser des Observables sans comprendre s'ils sont Cold ou Hot provoque des exécutions en double ou des comportements inattendus.

### ❌ Mauvais exemple

```ts
import { ajax } from 'rxjs/ajax';

// Cold Observable - une requête HTTP est exécutée pour chaque souscription
const data$ = ajax.getJSON('https://api.example.com/data');

data$.subscribe(console.log); // Requête 1
data$.subscribe(console.log); // Requête 2 (duplication inutile)
```

### ✅ Bon exemple

```ts
import { ajax } from 'rxjs/ajax';
import { shareReplay } from 'rxjs';

// Convertir en Hot Observable pour partager
const data$ = ajax.getJSON('https://api.example.com/data').pipe(
  shareReplay({ bufferSize: 1, refCount: true })
);

data$.subscribe(console.log); // Requête 1
data$.subscribe(console.log); // Utilise le résultat en cache
```

### Explication

- Cold Observable : exécuté pour chaque souscription (`of`, `from`, `fromEvent`, `ajax`, etc.)
- Hot Observable : exécuté indépendamment des souscriptions (`Subject`, Observable multicast, etc.)
- Convertir Cold en Hot avec `share` / `shareReplay`


## 7. Mélange inapproprié de Promise et Observable

### Problème

Mélanger Promise et Observable sans conversion appropriée rend la gestion d'erreurs et l'annulation incomplètes.

### ❌ Mauvais exemple

```ts
import { from } from 'rxjs';

async function fetchData(): Promise<string> {
  return 'data';
}

// Utiliser Promise directement
from(fetchData()).subscribe(data => {
  fetchData().then(moreData => { // Promise imbriquée
    console.log(data, moreData);
  });
});
```

### ✅ Bon exemple

```ts
import { from } from 'rxjs';
import { switchMap } from 'rxjs';

async function fetchData(): Promise<string> {
  return 'data';
}

// Convertir Promise en Observable pour unifier
from(fetchData()).pipe(
  switchMap(() => from(fetchData()))
).subscribe(moreData => {
  console.log(moreData);
});
```

### Explication

- Convertir Promise en Observable avec `from`
- Traiter de manière unifiée dans le pipeline Observable
- Gestion d'erreurs et annulation facilitées


## 8. Ignorer le backpressure

### Problème

Traiter des événements à haute fréquence sans contrôle dégrade les performances.

### ❌ Mauvais exemple

```ts
import { fromEvent } from 'rxjs';

// Traiter les événements d'entrée tels quels
fromEvent(document.getElementById('search'), 'input').subscribe(event => {
  // Appel API à chaque saisie (surcharge)
  searchAPI((event.target as HTMLInputElement).value);
});

function searchAPI(query: string): void {
  console.log('Searching for:', query);
}
```

### ✅ Bon exemple

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, distinctUntilChanged, map, switchMap } from 'rxjs';

// Appliquer debounce et annulation
fromEvent(document.getElementById('search'), 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300), // Attendre 300ms
  distinctUntilChanged(), // Uniquement quand la valeur change
  switchMap(query => searchAPI(query)) // Annuler les anciennes requêtes
).subscribe(results => {
  console.log('Results:', results);
});
```

### Explication

- Attendre un certain temps avec `debounceTime`
- Limiter la fréquence maximale avec `throttleTime`
- Exclure les doublons avec `distinctUntilChanged`
- Annuler les anciennes requêtes avec `switchMap`


## 9. Suppression d'erreurs

### Problème

Ne pas gérer correctement les erreurs rend le débogage difficile et dégrade l'expérience utilisateur.

### ❌ Mauvais exemple

```ts
import { ajax } from 'rxjs/ajax';
import { catchError } from 'rxjs';
import { of } from 'rxjs';

// Ignorer les erreurs
ajax.getJSON('https://api.example.com/data').pipe(
  catchError(() => of(null)) // L'information d'erreur est perdue
).subscribe(data => {
  console.log(data); // Si null, cause inconnue
});
```

### ✅ Bon exemple

```ts
import { ajax } from 'rxjs/ajax';
import { catchError } from 'rxjs';
import { of } from 'rxjs';

interface ApiResponse {
  data: unknown;
  error?: string;
}

ajax.getJSON<ApiResponse>('https://api.example.com/data').pipe(
  catchError((error: unknown) => {
    console.error('API Error:', error);
    // Notifier l'utilisateur
    showErrorToast('Échec de la récupération des données');
    // Retourner une valeur alternative incluant l'information d'erreur
    const message = error instanceof Error ? error.message : String(error);
    return of({ data: null, error: message } as ApiResponse);
  })
).subscribe((response) => {
  if (response.error) {
    console.log('Fallback mode due to:', response.error);
  }
});

function showErrorToast(message: string): void {
  console.log('Toast:', message);
}
```

### Explication

- Enregistrer les erreurs dans les logs
- Fournir un retour à l'utilisateur
- Retourner une valeur alternative incluant l'information d'erreur
- Considérer une stratégie de retry (`retry`, `retryWhen`)


## 10. Fuites de souscriptions d'événements DOM

### Problème

Ne pas libérer correctement les écouteurs d'événements DOM provoque des fuites mémoire.

### ❌ Mauvais exemple

```ts
import { fromEvent } from 'rxjs';

class Widget {
  private button: HTMLButtonElement;

  constructor() {
    this.button = document.createElement('button');

    // Enregistrer un écouteur d'événements
    fromEvent(this.button, 'click').subscribe(() => {
      console.log('clicked');
    });

    // Pas de désinscription
  }

  destroy(): void {
    this.button.remove();
    // L'écouteur reste en mémoire
  }
}
```

### ✅ Bon exemple

```ts
import { fromEvent, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class Widget {
  private button: HTMLButtonElement;
  private readonly destroy$ = new Subject<void>();

  constructor() {
    this.button = document.createElement('button');

    fromEvent(this.button, 'click').pipe(
      takeUntil(this.destroy$)
    ).subscribe(() => {
      console.log('clicked');
    });
  }

  destroy(): void {
    this.destroy$.next();
    this.destroy$.complete();
    this.button.remove();
  }
}
```

### Explication

- Désinscription garantie avec le pattern `takeUntil`
- Déclencher `destroy$` lors de la destruction du composant
- Libérer les écouteurs avant de supprimer l'élément DOM


## 11. Manque de sécurité de type (utilisation excessive de any)

### Problème

L'utilisation excessive de `any` désactive la vérification de type de TypeScript, augmentant les risques d'erreurs au runtime.

### ❌ Mauvais exemple

```ts
import { Observable } from 'rxjs';
import { map } from 'rxjs';

function fetchUser(): Observable<any> {
  return new Observable(subscriber => {
    subscriber.next({ name: 'John', age: 30 });
  });
}

// Pas de vérification de type
fetchUser().pipe(
  map(user => user.naem) // Faute de frappe ! Non détectée jusqu'au runtime
).subscribe(console.log);
```

### ✅ Bon exemple

```ts
import { Observable } from 'rxjs';
import { map } from 'rxjs';

interface User {
  name: string;
  age: number;
}

function fetchUser(): Observable<User> {
  return new Observable<User>(subscriber => {
    subscriber.next({ name: 'John', age: 30 });
  });
}

// Vérification de type active
fetchUser().pipe(
  map(user => user.name) // Erreur détectée à la compilation
).subscribe(console.log);
```

### Explication

- Définir des interfaces ou des alias de types
- Spécifier explicitement le paramètre de type de `Observable<T>`
- Utiliser pleinement l'inférence de type de TypeScript


## 12. Sélection d'opérateur inappropriée

### Problème

Utiliser un opérateur inadapté à l'objectif peut être inefficace ou provoquer un comportement inattendu.

### ❌ Mauvais exemple

```ts
import { fromEvent } from 'rxjs';
import { mergeMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Recherche à chaque clic (les anciennes requêtes ne sont pas annulées)
fromEvent(document.getElementById('search-btn'), 'click').pipe(
  mergeMap(() => ajax.getJSON('https://api.example.com/search'))
).subscribe(console.log);
```

### ✅ Bon exemple

```ts
import { fromEvent } from 'rxjs';
import { switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Traiter uniquement la dernière requête (annulation automatique des anciennes)
fromEvent(document.getElementById('search-btn'), 'click').pipe(
  switchMap(() => ajax.getJSON('https://api.example.com/search'))
).subscribe(console.log);
```

### Utilisation des principaux opérateurs d'ordre supérieur

| Opérateur | Usage |
|---|---|
| `switchMap` | Traiter uniquement le dernier flux (recherche, auto-complétion) |
| `mergeMap` | Traitement parallèle (ordre non important) |
| `concatMap` | Traitement séquentiel (ordre important) |
| `exhaustMap` | Ignorer les nouvelles entrées pendant l'exécution (anti-spam de bouton) |

### Explication

- Comprendre le comportement de chaque opérateur
- Sélection appropriée selon le cas d'usage
- Voir [Opérateurs de transformation](/fr/guide/operators/transformation/) pour plus de détails


## 13. Complexification excessive

### Problème

Complexifier excessivement avec RxJS des traitements qui pourraient être simples.

### ❌ Mauvais exemple

```ts
import { Observable, of } from 'rxjs';
import { map, mergeMap, toArray } from 'rxjs';

// Complexifier une simple transformation de tableau avec RxJS
function doubleNumbers(numbers: number[]): Observable<number[]> {
  return of(numbers).pipe(
    mergeMap(arr => of(...arr)),
    map(n => n * 2),
    toArray()
  );
}
```

### ✅ Bon exemple

```ts
import { fromEvent } from 'rxjs';
import { map } from 'rxjs';

// Le traitement de tableaux est suffisant avec du JavaScript normal
function doubleNumbers(numbers: number[]): number[] {
  return numbers.map(n => n * 2);
}

// Utiliser RxJS pour le traitement asynchrone et événementiel
const button = document.getElementById('calc-btn') as HTMLButtonElement;
const numbers = [1, 2, 3, 4, 5];

fromEvent(button, 'click').pipe(
  map(() => doubleNumbers(numbers))
).subscribe(result => console.log(result));
```

### Explication

- Utiliser RxJS pour le traitement asynchrone et les flux d'événements
- Le traitement synchrone de tableaux est suffisant avec du JavaScript normal
- Considérer l'équilibre entre complexité et bénéfices


## 14. Modification d'état dans subscribe

### Problème

Modifier directement l'état dans `subscribe` rend les tests difficiles et devient une source de bugs.

### ❌ Mauvais exemple

```ts
import { interval } from 'rxjs';

class Counter {
  count = 0;

  start(): void {
    interval(1000).subscribe(() => {
      this.count++; // Modification d'état dans subscribe
      this.updateUI();
    });
  }

  updateUI(): void {
    console.log('Count:', this.count);
  }
}
```

### ✅ Bon exemple

```ts
import { interval, BehaviorSubject } from 'rxjs';
import { scan, tap } from 'rxjs';

class Counter {
  private readonly count$ = new BehaviorSubject<number>(0);

  start(): void {
    interval(1000).pipe(
      scan(acc => acc + 1, 0),
      tap(count => this.count$.next(count))
    ).subscribe();

    // L'UI s'abonne à count$
    this.count$.subscribe(count => this.updateUI(count));
  }

  updateUI(count: number): void {
    console.log('Count:', count);
  }
}
```

### Explication

- Gérer l'état avec `BehaviorSubject` ou `scan`
- Utiliser `subscribe` comme déclencheur
- Conception testable et réactive


## 15. Absence de tests

### Problème

Déployer du code RxJS en production sans tests augmente les risques de régression.

### ❌ Mauvais exemple

```ts
import { interval } from 'rxjs';
import { map, filter } from 'rxjs';

// Déploiement sans tests
export function getEvenNumbers() {
  return interval(1000).pipe(
    filter(n => n % 2 === 0),
    map(n => n * 2)
  );
}
```

### ✅ Bon exemple

```ts
import { TestScheduler } from 'rxjs/testing';
import { getEvenNumbers } from './numbers';

describe('getEvenNumbers', () => {
  let scheduler: TestScheduler;

  beforeEach(() => {
    scheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('should emit only even numbers doubled', () => {
    scheduler.run(({ expectObservable }) => {
      const expected = '1s 0 1s 4 1s 8';
      expectObservable(getEvenNumbers()).toBe(expected);
    });
  });
});
```

### Explication

- Effectuer des tests marble avec `TestScheduler`
- Possibilité de tester le traitement asynchrone de manière synchrone
- Voir [Méthodes de test](/fr/guide/testing/unit-tests) pour plus de détails


## Résumé

Comprendre et éviter ces 15 anti-patterns vous permettra d'écrire du code RxJS plus robuste et maintenable.

## Références

Cette collection d'anti-patterns a été créée en référence aux sources fiables suivantes.

### Documentation officielle et dépôts
- **[Documentation officielle RxJS](https://rxjs.dev/)** - Référence officielle des opérateurs et de l'API
- **[GitHub Issue #5931](https://github.com/ReactiveX/rxjs/issues/5931)** - Discussion sur le problème de fuite mémoire de shareReplay

### Anti-patterns et meilleures pratiques
- **[RxJS in Angular - Antipattern 1: Nested subscriptions](https://www.thinktecture.com/en/angular/rxjs-antipattern-1-nested-subs/)** - Thinktecture AG
- **[RxJS in Angular - Antipattern 2: Stateful Streams](https://www.thinktecture.com/en/angular/rxjs-antipattern-2-state/)** - Thinktecture AG
- **[RxJS Best Practices in Angular 16 (2025)](https://www.infoq.com/articles/rxjs-angular16-best-practices/)** - InfoQ (mai 2025)
- **[RxJS: Why memory leaks occur when using a Subject](https://angularindepth.com/posts/1433/rxjs-why-memory-leaks-occur-when-using-a-subject)** - Angular In Depth
- **[RxJS Antipatterns](https://brianflove.com/posts/2017-11-01-ngrx-anti-patterns/)** - Brian Love

### Ressources supplémentaires
- **[Learn RxJS](https://www.learnrxjs.io/)** - Guide pratique des opérateurs et patterns
- **[RxJS Marbles](https://rxmarbles.com/)** - Compréhension visuelle des opérateurs

## Utiliser pour la revue de code

Vérifiez si votre code correspond à un anti-pattern.

👉 **[Checklist d'évitement des anti-patterns](./checklist)** - Révisez votre code avec 15 points de vérification

Chaque point de vérification permet d'accéder directement aux détails de l'anti-pattern correspondant sur cette page.

## Prochaines étapes

- **[Gestion des erreurs](/fr/guide/error-handling/strategies)** - Apprendre des stratégies de gestion d'erreurs plus détaillées
- **[Méthodes de test](/fr/guide/testing/unit-tests)** - Maîtriser les méthodes efficaces de test du code RxJS
- **[Comprendre les opérateurs](/fr/guide/operators/)** - Apprendre l'utilisation détaillée de chaque opérateur

Intégrez ces meilleures pratiques dans votre codage quotidien pour écrire du code RxJS de haute qualité !
