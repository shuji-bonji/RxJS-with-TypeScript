---
description: "Le marble testing est une méthode qui permet de tester les flux asynchrones de RxJS en les représentant visuellement avec des chaînes de caractères. Explication de la différence entre Cold et Hot Observable, règles de notation marble, utilisation de TestScheduler, et méthodes de vérification des traitements asynchrones complexes avec des exemples de code TypeScript."
---

# Introduction au marble testing

Dans RxJS, une méthode appelée "marble testing" est disponible pour tester le comportement des flux asynchrones **tout en les représentant visuellement**.

Ici, nous apprendrons les bases du marble testing à travers des exemples simples.

## Qu'est-ce que la notation marble ?

La notation marble est une méthode pour représenter **le passage du temps et l'occurrence d'événements** avec une chaîne de caractères.

### Règles de base

| Symbole | Signification |
|:----|:----|
| `-` | Passage du temps (avance d'1 frame) |
| `a`, `b`, `c` | Valeur émise (caractère arbitraire) |
| `|` | Complétion (complete) |
| `#` | Erreur (error) |

Par exemple

```text
--a-b--c-|
```
Cela signifie :
- Attend 2 frames puis émet `a`
- 1 frame plus tard, émet `b`
- 2 frames plus tard, émet `c`
- 1 frame de plus puis se termine

## Différence entre Cold et Hot

### Cold Observable

Les Cold Observables "rejouent depuis le début pour chaque souscription".

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

      expectObservable(source$).toBe('--a--b--c|');
    });
  });
});

```

### Hot Observable

Les Hot Observables sont des flux "déjà en cours".
Lors d'une souscription en cours de route, seules les valeurs à partir de ce moment peuvent être reçues.

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
    testScheduler.run(({ hot, expectObservable }) => {
      const source$ = hot('--a--b--c|');

      expectObservable(source$, '----^').toBe('-----b--c|');
    });
  });
});

```

## Exemple simple de marble testing

Par exemple, pour tester l'opérateur `debounceTime`

```ts
import { debounceTime } from 'rxjs';
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
      const source$ = cold('--a--b----c|');
      const result$ = source$.pipe(
        debounceTime(20)
      );

      const expected =    '-----------(c|)';

      expectObservable(result$).toBe(expected, { c: 'c' });
    });
  });
});

```

Ici, nous vérifions que seul le `c` émis en dernier est produit en sortie.

## Points d'attention

- Un caractère dans la notation marble représente par défaut **1 frame (10ms)** (configurable selon l'environnement)
- Les opérateurs dépendants du temps comme `debounceTime`, `delay`, `interval` **sont très compatibles avec le marble testing**
- Utilisez `expectObservable` pour vérifier la sortie du flux
- `expectSubscriptions` est une fonctionnalité avancée pour vérifier le timing des souscriptions, mais n'est pas couverte ici

## Résumé

Le marble testing est une méthode très puissante qui permet de **visualiser et comprendre intuitivement** les tests de code RxJS.

- **Être conscient de la différence entre Cold et Hot**
- **Représenter le passage du temps et les événements avec des chaînes de caractères**
- **Des tests clairs deviennent possibles même pour des flux asynchrones complexes**

Commençons par pratiquer avec des marble tests simples !
