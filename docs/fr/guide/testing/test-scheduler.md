---
description: "TestScheduler est un outil puissant qui permet de tester les opérateurs temporels de RxJS en utilisant le temps virtuel. Explication de la notation marble, création d'Observables Cold et Hot, méthodes pour faire avancer le temps, tests de debounceTime et delay, implémentation type-safe avec TypeScript."
---

# Tests avec TestScheduler

Le `TestScheduler` de RxJS est un outil puissant pour tester avec précision les opérateurs basés sur le temps. Ce chapitre explique systématiquement les méthodes de test utilisant TestScheduler.

## Qu'est-ce que TestScheduler ?

Normalement, les Observables fonctionnent en fonction du temps. Par exemple, `delay()` et `debounceTime()` sont des opérateurs qui attendent un certain temps.
Attendre réellement dans les tests est inefficace, c'est pourquoi le mécanisme permettant de tester instantanément en utilisant le temps virtuel est `TestScheduler`.

> [!TIP]
> TestScheduler utilise le "temps virtuel", éliminant ainsi le besoin d'attente en temps réel.

## Structure de base de TestScheduler

```ts
import { TestScheduler } from 'rxjs/testing';
import { describe, it, beforeEach, expect } from 'vitest';

describe('Base de TestScheduler', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('test simple', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--b--c|');
      const expected =  '--a--b--c|';

      expectObservable(source$).toBe(expected);
    });
  });
});
```

- `cold()` : Crée un Cold Observable où le flux commence indépendamment pour chaque souscription
- `hot()` : Crée un Hot Observable où le flux est déjà en cours
- `expectObservable()` : Vérifie la sortie de l'Observable avec la notation marble

## Cold Observable et Hot Observable

|Type|Caractéristique|Usage|
|:---|:---|:---|
|Cold Observable|Les données commencent à circuler depuis le début à chaque souscription|Requêtes HTTP, etc.|
|Hot Observable|Le flux de données a déjà commencé et est partagé avec les souscripteurs|Événements utilisateur, WebSocket, etc.|

## Bases de la notation marble

La notation marble est une méthode pour représenter le passage du temps d'un Observable sous forme de chaîne de caractères.

|Symbole|Signification|
|:---|:---|
|`-`|Passage du temps (1 frame)|
|`a`, `b`, `c`|Valeur émise|
|`|`|Complétion|
|`#`|Erreur|
|`() `|Émission simultanée de plusieurs valeurs (événements multiples)|

#### Exemple

```
--a--b--c|    // Émet 'a' après 2 frames, puis 'b', 'c', et se termine
```

## Exemple de test utilisant le temps virtuel

### Test de debounceTime

```ts
import { describe, it, expect, beforeEach } from 'vitest';
import { debounceTime, map } from 'rxjs';
import { TestScheduler } from 'rxjs/testing';

describe('Test utilisant le temps virtuel', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('test de l\'opérateur debounceTime', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--b----c|');
      const result$ = source$.pipe(
        debounceTime(20),
        map(x => x.toUpperCase())
      );
      const expected =    '-----------(C|)';  // ←Ceci !

      expectObservable(result$).toBe(expected, { B: 'B', C: 'C' });
    });
  });
});
```

## Test de gestion d'erreurs

```ts
import { describe, it, expect, beforeEach } from 'vitest';
import { catchError} from 'rxjs';
import { TestScheduler } from 'rxjs/testing';
import { of } from 'rxjs';

describe('Test de gestion d\'erreurs', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('capture l\'erreur avec catchError', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--#');
      const result$ = source$.pipe(
        catchError(() => of('X'))
      );

      const expected =    '--a--(X|)';

      expectObservable(result$).toBe(expected);
    });
  });
});
```

## Résumé

- TestScheduler permet de tester sans attendre le temps réel
- Comprendre et utiliser la différence entre Cold/Hot Observable
- Utiliser la notation marble pour visualiser le passage du temps
- Même les flux asynchrones complexes peuvent être testés avec précision

> [!NEXT]
> Ensuite, vous apprendrez le marble testing plus avancé (personnalisation des chaînes marble et combinaison de plusieurs flux).
