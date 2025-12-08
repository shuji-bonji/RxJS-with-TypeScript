---
description: "Les tests unitaires RxJS utilisent des techniques synchrones, asynchrones et de contrôle du temps. Ce guide explique comment construire une stratégie de test robuste avec TestScheduler, marble testing, mocks et stubs. Exemples d'implémentation avec Jasmine/Jest et meilleures pratiques pour les tests asynchrones."
---

# Tests unitaires RxJS

Le code utilisant RxJS comporte beaucoup de traitement asynchrone, ce qui nécessite une approche différente des méthodes de test traditionnelles. Ce guide explique les techniques de base aux techniques avancées pour tester efficacement le code utilisant RxJS.

## Tests d'Observables synchrones

Commençons par le cas le plus simple : tester un Observable qui se termine de manière synchrone.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect } from 'vitest';

// Fonction à tester
function doubleValues(input$: Observable<number>) : Observable<number>{
  return input$.pipe(
    map(x => x * 2)
  );
}

describe('Test d\'Observable basique', () => {
  it('double les valeurs', () => {
    // Observable de test
    const source$ = of(1, 2, 3);
    const result$ = doubleValues(source$);

    // Résultat attendu
    const expected = [2, 4, 6];
    const actual: number[] = [];

    // Exécution et vérification
    result$.subscribe({
      next: (value) => actual.push(value),
      complete: () => {
        expect(actual).toEqual(expected);
      }
    });
  });
});
```

## Comment tester les Observables asynchrones

Pour les Observables asynchrones, utilisez le support asynchrone du framework de test.

```ts
import { Observable, timer } from 'rxjs';
import { map, take } from 'rxjs';
import { describe, it, expect } from 'vitest';

// Fonction asynchrone à tester
function getDelayedValues(): Observable<number> {
  return timer(0, 100).pipe(
    map(x => x + 1),
    take(3)
  );
}

describe('Test d\'Observable asynchrone', () => {
  it('reçoit les valeurs asynchrones dans l\'ordre', (done: Function) => {
    const result$ = getDelayedValues();
    const expected = [1, 2, 3];
    const actual: number[] = [];

    result$.subscribe({
      next: (value) => actual.push(value),
      complete: () => {
        expect(actual).toEqual(expected);
        done();
      }
    });
  });
});
```

## Tests asynchrones par conversion en Promise

Vous pouvez également convertir un Observable en Promise en utilisant `firstValueFrom()` ou `lastValueFrom()` et profiter du async/await moderne de JS/TS.

```ts
import { Observable, of } from 'rxjs';
import { map, delay, toArray } from 'rxjs';
import { describe, it, expect } from 'vitest';
import { lastValueFrom } from 'rxjs';

// Fonction à tester
function processWithDelay(input$: Observable<number>) {
  return input$.pipe(
    map(x => x * 10),
    delay(100),
    toArray()
  );
}

describe('Test utilisant la conversion en Promise', () => {
  it('attend le traitement avec délai avant de vérifier', async () => {
    const source$ = of(1, 2, 3);
    const result$ = processWithDelay(source$);

    // Convertit l'Observable en promise
    const result = await lastValueFrom(result$);

    // Résultat attendu
    expect(result).toEqual([10, 20, 30]);
  });
});
```

## Utilisation de TestScheduler

RxJS fournit un scheduler spécial appelé `TestScheduler`, qui permet de tester efficacement les opérateurs basés sur le temps.

```ts
import { TestScheduler } from 'rxjs/testing';
import { map, debounceTime } from 'rxjs';
import { describe, it, beforeEach } from 'vitest';

describe('Utilisation de TestScheduler', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('test de debounceTime', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source = cold('a--b--c--d|', { a: 1, b: 2, c: 3, d: 4 });
      const result = source.pipe(
        debounceTime(20),
        map(x => x * 10)
      );

      const expected = '----------(d|)';

      expectObservable(result).toBe(expected, { d: 40 });
    });
  });
});
```

> [!NOTE]
> Notation marble testing
> Lors de l'utilisation de `TestScheduler`, utilisez les diagrammes marble pour représenter le passage du temps.

## Rendre le temps manipulable

Pour tester le code dépendant du temps (delay, debounceTime, etc.), utilisez `TestScheduler` pour contrôler le temps.

```ts
import { TestScheduler } from 'rxjs/testing';
import { interval } from 'rxjs';
import { take, map } from 'rxjs';
import { describe, it, beforeEach } from 'vitest';

describe('Contrôle du temps', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('test avec avance rapide du temps', () => {
    testScheduler.run(({ expectObservable }) => {
      const source = interval(1000).pipe(
        take(3),
        map(x => x + 1)
      );

      // Prend réellement 3 secondes, mais s'exécute instantanément dans l'environnement de test
      const expected = '1s a 999ms b 999ms (c|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source).toBe(expected, values);
    });
  });
});
```

## Test de gestion d'erreurs (version TestScheduler)

Il est également important de tester le comportement de l'Observable lorsqu'une erreur se produit.

```ts
import { TestScheduler } from 'rxjs/testing';
import { throwError, of } from 'rxjs';
import { catchError } from 'rxjs';

describe('Test de gestion d\'erreurs', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('cas où l\'Observable notifie une erreur', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source = cold('  --a--b--#');
      const expected =     '--a--b--#';

      expectObservable(source).toBe(expected);
    });
  });

  it('cas où l\'erreur est capturée avec catchError et remplacée par une valeur', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source = cold('  --a--b--#');
      const handled = source.pipe(
        catchError(() => of('X'))
      );

      const expected =     '--a--b--(X|)';

      expectObservable(handled).toBe(expected);
    });
  });
});
```

## Marble testing

Pour tester des flux complexes, utilisez les diagrammes marble pour exprimer intuitivement les valeurs attendues du test.

### Hot Observable vs Cold Observable

TestScheduler peut créer deux types d'Observables : hot et cold. Il est important de comprendre cette différence pour tester.

```ts
import { TestScheduler } from 'rxjs/testing';
import { Subject } from 'rxjs';
import { describe, it, beforeEach, expect } from 'vitest';

describe('Test Hot vs Cold Observable', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Cold Observable génère un flux indépendant pour chaque souscription', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      // Cold Observable (indépendant pour chaque souscripteur)
      const source = cold('--a--b--c|', { a: 1, b: 2, c: 3 });

      // Première souscription
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });

      // Deuxième souscription (commence depuis le début)
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });
    });
  });

  it('Hot Observable partage le flux entre les souscripteurs', () => {
    testScheduler.run(({ hot, expectObservable }) => {
      // Hot Observable (partagé entre les souscripteurs)
      const source = hot('--a--b--c|', { a: 1, b: 2, c: 3 });

      // Souscription tardive (ne reçoit que les valeurs après le début de la souscription)
      // expectObservable(source, '-----^---').toBe('-----b--c|', { b: 2, c: 3 });
      expectObservable(source, '----^').toBe('-----b--c|', { b: 2, c: 3 });

      // Souscription dès le début (reçoit toutes les valeurs)
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });
    });
  });

  it('test de Hot Observable avec un Subject réel', () => {
    // Version non-TestScheduler
    const subject = new Subject<number>();
    const values1: number[] = [];
    const values2: number[] = [];

    // Premier souscripteur
    const subscription1 = subject.subscribe(val => values1.push(val));

    // Émet des valeurs
    subject.next(1);
    subject.next(2);

    // Deuxième souscripteur (en cours de route)
    const subscription2 = subject.subscribe(val => values2.push(val));

    // Émet plus de valeurs
    subject.next(3);
    subject.complete();

    // Vérification
    expect(values1).toEqual([1, 2, 3]);
    expect(values2).toEqual([3]); // Seulement les valeurs après le début de la souscription

    // Nettoyage
    subscription1.unsubscribe();
    subscription2.unsubscribe();
  });
});
```

> [!NOTE]
> Les Cold Observables génèrent des données indépendamment à chaque souscription, tandis que les Hot Observables partagent et diffusent les données.

## Utilisation de mocks et stubs

### Mock des services dépendants

Lors du test de services utilisant RxJS, il est courant de mocker les dépendances externes.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect, vi } from 'vitest';

type User = {
  id: number;
  name: string;
  active: boolean;
}

// Service à tester
class UserService {
  constructor(private apiService: { fetchUsers: Function }) {}

  getUsers(): Observable<User[]> {
    return this.apiService.fetchUsers().pipe(
      map((users: User[]) => users.filter(user => user.active))
    );
  }
}

describe('Test de service', () => {
  it('filtre uniquement les utilisateurs actifs', () => {
    // Service API mocké
    const mockApiService = {
      fetchUsers: vi.fn().mockReturnValue(of([
        { id: 1, name: 'Tanaka', active: true },
        { id: 2, name: 'Sato', active: false },
        { id: 3, name: 'Yamada', active: true }
      ]))
    };

    const userService = new UserService(mockApiService);
    const result$ = userService.getUsers();

    // Vérification
    result$.subscribe(users => {
      expect(users.length).toBe(2);
      expect(users[0].name).toBe('Tanaka');
      expect(users[1].name).toBe('Yamada');
      expect(mockApiService.fetchUsers).toHaveBeenCalledTimes(1);
    });
  });
});
```

### Utilisation de Stub

Un stub est un objet simple qui imite les données ou API externes dont dépend le code à tester.
Il élimine les dépendances aux ressources externes et permet aux tests de fonctionner indépendamment.
Il ne fait que retourner des valeurs fixes, sans logique interne.

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect } from 'vitest';

type User = {
  id: number;
  name: string;
  active: boolean;
};

// Service à tester
class UserService {
  constructor(private apiService: { fetchUsers: Function }) {}

  getActiveUsers() {
    return this.apiService.fetchUsers().pipe(
      map((users: User[]) => users.filter(user => user.active))
    );
  }
}

describe('Test de UserService', () => {
  it('retourne uniquement les utilisateurs actifs', () => {
    // 🔹 Création du stub
    const stubApiService = {
      fetchUsers: () => of<User[]>([
        { id: 1, name: 'Tanaka', active: true },
        { id: 2, name: 'Sato', active: false },
        { id: 3, name: 'Yamada', active: true }
      ])
    };

    // Service à tester
    const userService = new UserService(stubApiService);

    // Vérification du résultat
    userService.getActiveUsers().subscribe((users: User[]) => {
      expect(users.length).toBe(2);
      expect(users[0].name).toBe('Tanaka');
      expect(users[1].name).toBe('Yamada');
    });
  });
});
```

## Espionnage de souscription

Vous pouvez utiliser des espions pour vérifier que les souscriptions sont effectuées correctement.

```ts
import { Subject } from 'rxjs';
import { describe, it, expect, vi } from 'vitest';

describe('Test de souscription', () => {
  it('souscrit avec les gestionnaires appropriés', () => {
    const subject = new Subject();

    // Créer des espions pour les gestionnaires
    const nextSpy = vi.fn();
    const errorSpy = vi.fn();
    const completeSpy = vi.fn();

    // Souscrire
    subject.subscribe({
      next: nextSpy,
      error: errorSpy,
      complete: completeSpy
    });

    // Émettre des valeurs
    subject.next('value1');
    subject.next('value2');
    subject.complete();

    // Vérification
    expect(nextSpy).toHaveBeenCalledTimes(2);
    expect(nextSpy).toHaveBeenCalledWith('value1');
    expect(nextSpy).toHaveBeenCalledWith('value2');
    expect(errorSpy).not.toHaveBeenCalled();
    expect(completeSpy).toHaveBeenCalledTimes(1);
  });
});
```

## Meilleures pratiques

|Meilleure pratique|Explication|
|---|---|
|Respecter le principe de responsabilité unique|Pour écrire du code testable, assurez-vous que chaque fonction ou classe a une seule responsabilité. Cela simplifie également les tests.|
|Mocker les dépendances externes|Mockez les dépendances externes comme les requêtes HTTP ou les timers pour effectuer des tests dans un environnement prévisible.|
|Utiliser la technique appropriée pour le code asynchrone|Pour les tests asynchrones, choisissez la méthode appropriée : TestScheduler, callback done(), ou async/await.|
|Utiliser le marble testing|Pour tester des flux complexes, utilisez les diagrammes marble pour exprimer intuitivement les valeurs attendues du test.

## Résumé

Les tests de code RxJS présentent des aspects différents du code JavaScript traditionnel, notamment la nature synchrone/asynchrone et le comportement dépendant du temps. En choisissant les bonnes techniques de test, vous pouvez développer du code réactif de haute qualité en toute confiance. Gardez particulièrement à l'esprit les points suivants :

- Tests de souscription simples pour les Observables synchrones
- TestScheduler ou conversion en Promise pour le traitement asynchrone
- Marble testing pour le code dépendant du temps
- Mocker les dépendances externes pour créer un environnement de test indépendant
- Suivre le principe de responsabilité unique et concevoir du code facile à tester

## 🔗 Sections connexes

- **[Erreurs courantes et solutions](/fr/guide/anti-patterns/common-mistakes#15-absence-de-tests)** - Vérifier les anti-patterns liés aux tests
- **[Utilisation de TestScheduler](/fr/guide/testing/test-scheduler)** - Usage plus détaillé de TestScheduler
- **[Marble testing](/fr/guide/testing/marble-testing)** - Techniques avancées de marble testing
