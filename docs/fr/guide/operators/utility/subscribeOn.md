---
description: "L'opérateur subscribeOn contrôle le timing de démarrage de l'abonnement d'un Observable en utilisant un planificateur spécifié, modifiant le contexte d'exécution du flux entier."
---

# subscribeOn - Contrôle du timing de démarrage de l'abonnement

L'opérateur `subscribeOn` **contrôle le timing de démarrage de l'abonnement et le contexte d'exécution en utilisant un planificateur spécifié**. Il affecte le timing d'exécution du flux entier.

## 🔰 Syntaxe et comportement de base

Spécifiez un planificateur pour rendre le démarrage de l'abonnement asynchrone.

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

console.log('Début');

of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler)
  )
  .subscribe(v => console.log('Valeur :', v));

console.log('Fin');

// Sortie :
// Début
// Fin
// Valeur : 1
// Valeur : 2
// Valeur : 3
```

Puisque le démarrage de l'abonnement est rendu asynchrone, l'appel `subscribe()` retourne immédiatement.

[🌐 Documentation officielle RxJS - subscribeOn](https://rxjs.dev/api/index/function/subscribeOn)

## 💡 Cas d'utilisation typiques

- **Initialisation lourde asynchrone** : Retarder le début du chargement des données
- **Prévention du gel de l'UI** : Rendre le démarrage de l'abonnement asynchrone pour maintenir la réactivité
- **Priorisation du traitement** : Contrôle du timing de démarrage de plusieurs flux
- **Contrôle du timing dans les tests** : Utilisation de TestScheduler pour le contrôle

## 🧪 Exemple de code pratique 1 : Initialisation lourde asynchrone

Exemple de démarrage asynchrone du chargement de données ou de l'initialisation.

```ts
import { Observable, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

// Création de l'interface utilisateur
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'subscribeOn - Initialisation lourde';
container.appendChild(title);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

function addLog(message: string, color: string = '#e3f2fd') {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;

  const now = new Date();
  const timestamp = now.toLocaleTimeString('fr-FR', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  logItem.textContent = `[${timestamp}] ${message}`;
  output.appendChild(logItem);
}

// Simulation d'un traitement d'initialisation lourd
const heavyInit$ = new Observable<string>(subscriber => {
  addLog('Début du chargement des données...', '#fff9c4');

  // Simulation d'un traitement lourd
  let sum = 0;
  for (let i = 0; i < 10000000; i++) {
    sum += i;
  }

  addLog('Chargement des données terminé', '#c8e6c9');
  subscriber.next(`Résultat : ${sum}`);
  subscriber.complete();
});

addLog('Démarrage de l\'abonnement (UI opérationnelle)', '#e3f2fd');

heavyInit$
  .pipe(
    subscribeOn(asyncScheduler)  // Rendre le démarrage de l'abonnement asynchrone
  )
  .subscribe({
    next: result => addLog(`Reçu : ${result}`, '#c8e6c9'),
    complete: () => addLog('Terminé', '#e3f2fd')
  });

addLog('Après la demande d\'abonnement (exécution continue immédiatement)', '#e3f2fd');
```

- Le démarrage de l'abonnement est rendu asynchrone, l'UI répond immédiatement
- Le traitement lourd s'exécute de manière asynchrone
- Le thread principal n'est pas bloqué

## 🧪 Exemple de code pratique 2 : Contrôle de priorité pour plusieurs flux

Exemple de contrôle du timing de démarrage de plusieurs flux.

```ts
import { interval, asyncScheduler, asapScheduler } from 'rxjs';
import { subscribeOn, take, tap } from 'rxjs';

// Création de l'interface utilisateur
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'subscribeOn - Contrôle de priorité';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '200px';
output2.style.overflow = 'auto';
container2.appendChild(output2);

function addLog2(message: string, color: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('fr-FR', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.style.padding = '3px';
  logItem.style.marginBottom = '2px';
  logItem.style.backgroundColor = color;
  logItem.style.fontSize = '12px';
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);
}

addLog2('Début', '#e3f2fd');

// Tâche haute priorité (asapScheduler)
interval(500)
  .pipe(
    take(3),
    subscribeOn(asapScheduler),
    tap(v => addLog2(`Haute priorité : ${v}`, '#c8e6c9'))
  )
  .subscribe();

// Tâche priorité normale (asyncScheduler)
interval(500)
  .pipe(
    take(3),
    subscribeOn(asyncScheduler),
    tap(v => addLog2(`Priorité normale : ${v}`, '#fff9c4'))
  )
  .subscribe();

addLog2('Demandes d\'abonnement terminées', '#e3f2fd');
```

- Contrôle de priorité avec différents planificateurs
- `asapScheduler` démarre l'exécution avant `asyncScheduler`

## 🆚 Différence avec observeOn

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, subscribeOn, tap } from 'rxjs';

// Exemple avec observeOn
console.log('=== observeOn ===');
console.log('1: Début');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('2: tap (synchrone)')),
    observeOn(asyncScheduler),
    tap(() => console.log('4: tap (asynchrone)'))
  )
  .subscribe(() => console.log('5: subscribe'));

console.log('3: Fin');

// Exemple avec subscribeOn
console.log('\n=== subscribeOn ===');
console.log('1: Début');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('3: tap (asynchrone)')),
    subscribeOn(asyncScheduler)
  )
  .subscribe(() => console.log('4: subscribe'));

console.log('2: Fin');
```

**Principales différences** :

| Élément | observeOn | subscribeOn |
|:---|:---|:---|
| **Portée** | Opérations suivantes uniquement | Flux entier |
| **Cible du contrôle** | Timing d'émission des valeurs | Timing de démarrage de l'abonnement |
| **Position de placement** | Important (le comportement change selon la position) | Même effet quelle que soit la position |
| **Utilisation multiple** | Le dernier s'applique | Le premier s'applique |

> [!NOTE]
> Pour plus de détails sur `observeOn`, voir [observeOn](./observeOn).

## ⚠️ Notes importantes

### 1. La position de placement n'a pas d'importance

`subscribeOn` a le même effet quel que soit l'endroit où il est placé dans le pipeline.

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn, map } from 'rxjs';

// Pattern 1 : Au début
of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler),
    map(x => x * 2)
  )
  .subscribe();

// Pattern 2 : À la fin
of(1, 2, 3)
  .pipe(
    map(x => x * 2),
    subscribeOn(asyncScheduler)
  )
  .subscribe();

// Les deux ont le même comportement
```

### 2. En cas d'utilisation multiple, le premier est appliqué

```ts
import { of, asyncScheduler, asapScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

of(1, 2, 3)
  .pipe(
    subscribeOn(asyncScheduler),  // Celui-ci est utilisé
    subscribeOn(asapScheduler)    // Celui-ci est ignoré
  )
  .subscribe();
```

Le planificateur du premier `subscribeOn` (`asyncScheduler`) est utilisé.

### 3. Pas d'effet sur certains Observables

`subscribeOn` n'a pas d'effet sur les Observables qui ont leur propre planificateur comme `interval` ou `timer`.

```ts
import { interval, asyncScheduler } from 'rxjs';
import { subscribeOn } from 'rxjs';

// ❌ subscribeOn n'a pas d'effet
interval(1000)
  .pipe(
    subscribeOn(asyncScheduler)  // interval utilise son propre planificateur
  )
  .subscribe();

// ✅ Spécifier le planificateur dans l'argument d'interval
interval(1000, asyncScheduler)
  .subscribe();
```

## Exemple de combinaison pratique

```ts
import { of, asyncScheduler, animationFrameScheduler } from 'rxjs';
import { subscribeOn, observeOn, map, tap } from 'rxjs';

console.log('Début');

of(1, 2, 3)
  .pipe(
    tap(() => console.log('Tap 1 (asynchrone)')),
    subscribeOn(asyncScheduler),        // Rendre le démarrage de l'abonnement asynchrone
    map(x => x * 2),
    observeOn(animationFrameScheduler), // Synchroniser l'émission des valeurs avec l'animation frame
    tap(() => console.log('Tap 2 (animation frame)'))
  )
  .subscribe(v => console.log('Valeur :', v));

console.log('Fin');

// Ordre d'exécution :
// Début
// Fin
// Tap 1 (asynchrone)
// Tap 1 (asynchrone)
// Tap 1 (asynchrone)
// Tap 2 (animation frame)
// Valeur : 2
// ... (suite)
```

## Guide d'utilisation

### Cas 1 : Vous voulez retarder le démarrage de l'abonnement
```ts
// → Utiliser subscribeOn
of(données)
  .pipe(subscribeOn(asyncScheduler))
  .subscribe();
```

### Cas 2 : Vous voulez rendre seulement certaines opérations asynchrones
```ts
// → Utiliser observeOn
of(données)
  .pipe(
    map(traitementLourd),
    observeOn(asyncScheduler),  // Seulement après le traitement lourd
    map(traitementLéger)
  )
  .subscribe();
```

### Cas 3 : Tout rendre asynchrone + contrôle supplémentaire de parties
```ts
// → Utiliser subscribeOn + observeOn ensemble
of(données)
  .pipe(
    subscribeOn(asyncScheduler),           // Tout rendre asynchrone
    map(traitement1),
    observeOn(animationFrameScheduler),    // Changer pour l'animation
    map(traitement2)
  )
  .subscribe();
```

## 📚 Opérateurs associés

- **[observeOn](./observeOn)** - Contrôle du timing d'émission des valeurs
- **[delay](./delay)** - Délai à temps fixe

## 📖 Documentation associée

- **[Contrôle du traitement asynchrone](/fr/guide/schedulers/async-control)** - Bases des planificateurs
- **[Types et utilisation des planificateurs](/fr/guide/schedulers/types)** - Détails de chaque planificateur

## ✅ Résumé

L'opérateur `subscribeOn` contrôle le timing de démarrage de l'abonnement et le contexte d'exécution.

- ✅ Rend le démarrage de l'abonnement du flux entier asynchrone
- ✅ Efficace pour l'initialisation lourde asynchrone
- ✅ Utile pour prévenir le gel de l'UI
- ✅ La position de placement n'a pas d'importance
- ⚠️ En cas d'utilisation multiple, le premier est appliqué
- ⚠️ Pas d'effet sur certains Observables
- ⚠️ Objectif différent de `observeOn`
