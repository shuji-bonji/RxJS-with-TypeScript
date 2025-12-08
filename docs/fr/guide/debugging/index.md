---
description: "Techniques de débogage RxJS : suivi des valeurs avec tap(), placement efficace de console.log, extension RxJS DevTools, création d'opérateurs de débogage personnalisés, mesure des performances. Présentation systématique des stratégies de débogage pratiques et méthodes d'identification des problèmes de flux de valeurs."
---

# Techniques de débogage RxJS

Le débogage RxJS nécessite une approche différente des techniques de débogage synchrones traditionnelles en raison de la nature asynchrone des streams.

Cette page fournit les stratégies de base pour déboguer les applications RxJS et une navigation vers des techniques de débogage détaillées.

## Vue d'ensemble des techniques de débogage

Le débogage RxJS peut être classé en 4 approches :

| Approche | Contenu | Page détaillée |
|----------|---------|----------------|
| **Stratégies de base** | Opérateur tap, outils de développement, RxJS DevTools | Expliqué sur cette page |
| **Scénarios courants** | 6 problèmes typiques : absence de flux de valeurs, fuites mémoire, erreurs manquées, etc. | [→ Détails](/fr/guide/debugging/common-scenarios) |
| **Outils personnalisés** | Streams nommés, opérateurs de débogage, mesure des performances | [→ Détails](/fr/guide/debugging/custom-tools) |
| **Performance** | Surveillance des abonnements, détection des réévaluations, vérification de l'utilisation mémoire, meilleures pratiques | [→ Détails](/fr/guide/debugging/performance) |

## Stratégies de débogage de base

### 1. Sortie de logs avec l'opérateur `tap`

L'opérateur `tap` est la technique de débogage la plus basique, permettant d'observer les valeurs d'un stream sans effets secondaires.

```ts
import { interval } from 'rxjs';
import { map, filter, tap } from 'rxjs';

interval(1000)
  .pipe(
    tap(value => console.log('🔵 Valeur originale:', value)),
    map(x => x * 2),
    tap(value => console.log('🟢 Après map:', value)),
    filter(x => x > 5),
    tap(value => console.log('🟡 Après filter:', value))
  )
  .subscribe(value => console.log('✅ Valeur finale:', value));

// Sortie:
// 🔵 Valeur originale: 0
// 🟢 Après map: 0
// 🔵 Valeur originale: 1
// 🟢 Après map: 2
// 🔵 Valeur originale: 2
// 🟢 Après map: 4
// 🔵 Valeur originale: 3
// 🟢 Après map: 6
// 🟡 Après filter: 6
// ✅ Valeur finale: 6
```

#### Points clés
- En insérant `tap` à chaque étape du pipeline, vous pouvez suivre le flux de données
- L'utilisation d'emojis et de labels améliore la visibilité des logs
- `tap` ne modifie pas les valeurs, vous pouvez donc insérer des logs de débogage en toute sécurité

### 2. Sortie d'informations de log détaillées

Pour obtenir des informations de débogage plus détaillées, utilisez un objet Observer.

```ts
import { of, throwError, concat } from 'rxjs';
import { tap } from 'rxjs';

const debug = (tag: string) =>
  tap({
    next: value => console.log(`[${tag}] next:`, value),
    error: error => console.error(`[${tag}] error:`, error),
    complete: () => console.log(`[${tag}] complete`)
  });

// Stream normal
of(1, 2, 3)
  .pipe(debug('Normal'))
  .subscribe();

// Sortie:
// [Normal] next: 1
// [Normal] next: 2
// [Normal] next: 3
// [Normal] complete

// Stream avec erreur
concat(
  of(1, 2),
  throwError(() => new Error('Erreur survenue'))
)
  .pipe(debug('Erreur'))
  .subscribe({
    error: () => {} // Gestion des erreurs
  });

// Sortie:
// [Erreur] next: 1
// [Erreur] next: 2
// [Erreur] error: Error: Erreur survenue
```

### 3. Vérification avec les outils de développement

Techniques de débogage utilisant les outils de développement du navigateur.

```ts
import { fromEvent, timer } from 'rxjs';
import { map, tap, debounceTime } from 'rxjs';

// Fonction helper pour le débogage
function tapDebugger<T>(label: string) {
  return tap<T>({
    next: value => {
      console.group(`🔍 ${label}`);
      console.log('Value:', value);
      console.log('Type:', typeof value);
      console.log('Timestamp:', new Date().toISOString());
      console.trace('Stack trace');
      console.groupEnd();
    }
  });
}

// Débogage d'événement de clic de bouton
const button = document.querySelector('button');
if (button) {
  fromEvent(button, 'click')
    .pipe(
      tapDebugger('Click Event'),
      debounceTime(300),
      tapDebugger('After Debounce'),
      map(() => ({ timestamp: Date.now() }))
    )
    .subscribe(data => console.log('📤 Envoi:', data));
}
```

#### Utilisation des outils de développement
- Grouper les logs avec `console.group()`
- Afficher la stack trace avec `console.trace()`
- Afficher les tableaux et objets de manière lisible avec `console.table()`
- Placer des breakpoints dans `tap`

### 4. Utilisation de RxJS DevTools

RxJS DevTools est un outil de débogage fourni comme extension de navigateur.

#### Installation
- Chrome: [RxJS DevTools - Chrome Web Store](https://chrome.google.com/webstore)
- Firefox: [RxJS DevTools - Firefox Add-ons](https://addons.mozilla.org/)

#### Fonctionnalités principales
- Visualisation de l'état d'abonnement des Observables
- Affichage chronologique des valeurs du stream
- Détection des fuites mémoire
- Analyse des performances

#### Exemple d'utilisation

```ts
import { interval } from 'rxjs';
import { take, map } from 'rxjs';

// Activer le débogage uniquement en environnement de développement
// La méthode de détection des variables d'environnement diffère selon l'outil de build
const isDevelopment =
  // Vite: import.meta.env.DEV
  // webpack: process.env.NODE_ENV === 'development'
  // Configuration manuelle: utiliser une variable globale
  typeof window !== 'undefined' && (window as any).__DEV__ === true;

const stream$ = interval(1000).pipe(
  take(5),
  map(x => x * 2)
);

if (isDevelopment) {
  // Rendre observable dans DevTools
  stream$.subscribe({
    next: value => console.log('DevTools:', value)
  });
}
```

## Techniques de débogage détaillées

Après avoir compris les stratégies de base, apprenez les techniques de débogage spécifiques sur les pages détaillées suivantes.

### Scénarios de débogage courants

6 problèmes typiques rencontrés en développement réel et leurs solutions

- Scénario 1 : Absence de flux de valeurs
- Scénario 2 : Sortie de valeurs différentes de celles attendues
- Scénario 3 : L'abonnement ne se termine pas (stream infini)
- Scénario 4 : Fuite mémoire (oubli de désabonnement)
- Scénario 5 : Erreurs non détectées
- Scénario 6 : Suivi du nombre de tentatives de retry

[→ Voir les scénarios de débogage courants](/fr/guide/debugging/common-scenarios)

### Outils de débogage personnalisés

Comment créer vos propres outils de débogage adaptés aux besoins de votre projet

- Débogage de streams nommés (tagStream)
- Création d'opérateurs de débogage personnalisés
- Opérateur de mesure des performances (measure)

[→ Voir les outils de débogage personnalisés](/fr/guide/debugging/custom-tools)

### Débogage des performances

Optimisation de l'application et meilleures pratiques

- Vérification et suivi du nombre d'abonnements
- Détection des réévaluations inutiles (shareReplay)
- Surveillance de l'utilisation mémoire
- Construction d'un environnement de débogage
- Débogage type-safe
- Configuration des limites d'erreur

[→ Voir le débogage des performances](/fr/guide/debugging/performance)

## Résumé

Le débogage RxJS peut être effectué efficacement en maîtrisant les points suivants.

### Stratégies de base
- ✅ Observer chaque étape du stream avec l'opérateur `tap`
- ✅ Sortie de logs détaillés avec les outils de développement
- ✅ Visualiser les streams avec RxJS DevTools

### Scénarios courants
- ✅ Absence de flux de valeurs → Vérifier l'oubli d'abonnement, les conditions de filtrage
- ✅ Valeurs différentes de celles attendues → Attention à l'ordre des opérateurs, au partage de références
- ✅ L'abonnement ne se termine pas → Utiliser `take` ou `takeUntil` pour les streams infinis
- ✅ Fuite mémoire → Désabonnement automatique avec le pattern `takeUntil`
- ✅ Erreurs manquées → Implémenter une gestion appropriée des erreurs

### Outils de débogage
- ✅ Débogage flexible avec des opérateurs de débogage personnalisés
- ✅ Suivre plusieurs streams avec des streams nommés
- ✅ Identifier les goulots d'étranglement avec la mesure des performances

### Performance
- ✅ Surveillance du nombre d'abonnements pour prévenir les fuites mémoire
- ✅ Éviter les recalculs inutiles avec `shareReplay`
- ✅ Vérifier régulièrement l'utilisation mémoire

En combinant ces techniques, vous pouvez déboguer efficacement les applications RxJS.

## Pages connexes

- [Gestion des erreurs](/fr/guide/error-handling/strategies) - Stratégies de traitement des erreurs
- [Techniques de test](/fr/guide/testing/unit-tests) - Méthodes de test RxJS
- [Anti-patterns RxJS](/fr/guide/anti-patterns/) - Erreurs courantes et solutions
- [Pipeline](/fr/guide/operators/pipeline) - Chaînage d'opérateurs
