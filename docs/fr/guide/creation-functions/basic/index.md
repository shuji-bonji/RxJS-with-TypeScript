---
description: "Cette section couvre les fonctions de création pour la création de base d'Observable, en utilisant of, from, fromEvent, interval et timer pour créer des Observables à partir de diverses sources de données incluant des valeurs simples, des tableaux, des Promises, des événements et des timers. C'est un concept important qui peut être implémenté avec la sécurité de type TypeScript et constitue la base de RxJS."
---

# Fonctions de création de base

Les fonctions de création les plus basiques et les plus fréquemment utilisées. Créez facilement des données, des tableaux, des événements et des Observables basés sur le temps.

## Que sont les fonctions de création de base ?

Les fonctions de création de base sont des fonctions permettant de créer un Observable unique à partir de diverses sources de données. Elles constituent l'ensemble de fonctions le plus fondamental de RxJS et sont utilisées dans presque tout le code RxJS.

Veuillez consulter le tableau ci-dessous pour connaître les caractéristiques et l'utilisation de chaque fonction de création.

## Principales fonctions de création de base

| Fonction | Description | Cas d'utilisation |
|----------|------|-------------|
| **[of](/fr/guide/creation-functions/basic/of)** | Émettre les valeurs spécifiées en séquence | Tests avec valeurs fixes, création de mocks |
| **[from](/fr/guide/creation-functions/basic/from)** | Convertir depuis un tableau, une Promise, etc. | Streaming de données existantes |
| **[fromEvent](/fr/guide/creation-functions/basic/fromEvent)** | Convertir les événements en Observable | Événements DOM, Node.js EventEmitter |
| **[interval](/fr/guide/creation-functions/basic/interval)** | Émettre en continu à intervalles spécifiés | Polling, exécution périodique |
| **[timer](/fr/guide/creation-functions/basic/timer)** | Commencer à émettre après un délai | Exécution différée, timeout |

## Critères d'utilisation

Le choix des fonctions de création de base est déterminé par le type de source de données.

### 1. Type de données

- **Valeurs statiques** : `of()` - Crée un Observable en spécifiant la valeur directement
- **Tableau ou Itérable** : `from()` - Convertit une collection existante en flux
- **Promise** : `from()` - Convertit un traitement asynchrone en Observable
- **Événement** : `fromEvent()` - Convertit un écouteur d'événement en Observable
- **Basé sur le temps** : `interval()`, `timer()` - Émet des valeurs basées sur le passage du temps

### 2. Moment de l'émission

- **Émettre immédiatement** : `of()`, `from()` - Commence à émettre des valeurs lors de l'abonnement
- **À l'occurrence d'un événement** : `fromEvent()` - Émet à chaque fois qu'un événement se produit
- **Émettre périodiquement** : `interval()` - Émet en continu à intervalles réguliers
- **Émettre après un délai** : `timer()` - Commence à émettre après un temps spécifié

### 3. Moment de l'achèvement

- **Terminer immédiatement** : `of()`, `from()` - Termine après que toutes les valeurs aient été émises
- **Ne jamais terminer** : `fromEvent()`, `interval()` - Continue jusqu'au désabonnement
- **Émettre une fois et terminer** : `timer(delay)` - Termine après l'émission d'une valeur

## Exemples d'utilisation pratique

### of() - Test avec des valeurs fixes

```typescript
import { of } from 'rxjs';

// Créer des données de test
const mockUser$ = of({ id: 1, name: 'Utilisateur Test' });

mockUser$.subscribe(user => console.log(user));
// Sortie: { id: 1, name: 'Utilisateur Test' }
```

### from() - Streaming d'un tableau

```typescript
import { from } from 'rxjs';
import { map } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

numbers$.pipe(
  map(n => n * 2)
).subscribe(console.log);
// Sortie: 2, 4, 6, 8, 10
```

### fromEvent() - Événement de clic

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(() => console.log('Bouton cliqué!'));
```

### interval() - Polling

```typescript
import { interval } from 'rxjs';
import { switchMap } from 'rxjs';

// Interroger l'API toutes les 5 secondes
interval(5000).pipe(
  switchMap(() => fetchData())
).subscribe(data => console.log('Mis à jour:', data));
```

### timer() - Exécution différée

```typescript
import { timer } from 'rxjs';

// Exécuter après 3 secondes
timer(3000).subscribe(() => console.log('3 secondes écoulées'));
```

## Attention aux fuites de mémoire

Un désabonnement correct est important lors de l'utilisation des fonctions de création de base.

> [!WARNING]
> `fromEvent()`, `interval()`, et `timer(delay, period)` périodique ne se termineront pas et doivent toujours être désabonnés avec `unsubscribe()` ou automatiquement avec `takeUntil()` ou similaire lorsque le composant est détruit.
>
> Note : `timer(delay)` sans second argument se terminera automatiquement après avoir émis une fois.

```typescript
import { fromEvent, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class MyComponent {
  private destroy$ = new Subject<void>();

  ngOnInit() {
    fromEvent(window, 'resize').pipe(
      takeUntil(this.destroy$)
    ).subscribe(() => console.log('Fenêtre redimensionnée'));
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

## Conversion de Cold vers Hot

Comme le montre le tableau ci-dessus, **toutes les fonctions de création de base génèrent des Cold Observables**. Chaque abonnement initie une exécution indépendante.

Cependant, vous pouvez **convertir un Cold Observable en Hot Observable** en utilisant les opérateurs de multicast suivants.

### Conditions et opérateurs pour la conversion en Hot

| Opérateur | Comportement | Cas d'utilisation |
|-------------|------|-------------|
| **share()** | Multicast + connexion/déconnexion automatique | Partager les requêtes HTTP entre plusieurs abonnés |
| **shareReplay(n)** | Mettre en cache les n dernières valeurs et les délivrer aux nouveaux abonnés | Mettre en cache les réponses API |
| **publish() + connect()** | Lancer manuellement le multicast | Démarrer l'exécution quand les abonnés sont prêts |
| **multicast(subject)** | Multicast avec un Subject personnalisé | Quand un contrôle avancé est nécessaire |

### Exemple pratique

```typescript
import { interval } from 'rxjs';
import { take, share } from 'rxjs';

// ❄️ Cold - Timer indépendant pour chaque abonnement
const cold$ = interval(1000).pipe(take(3));

cold$.subscribe(val => console.log('A:', val));
setTimeout(() => {
  cold$.subscribe(val => console.log('B:', val));
}, 1500);

// Sortie:
// A: 0 (après 0s)
// A: 1 (après 1s)
// B: 0 (après 1.5s) ← B démarre indépendamment depuis 0
// A: 2 (après 2s)
// B: 1 (après 2.5s)

// 🔥 Hot - Partage le timer entre les abonnés
const hot$ = interval(1000).pipe(take(3), share());

hot$.subscribe(val => console.log('A:', val));
setTimeout(() => {
  hot$.subscribe(val => console.log('B:', val));
}, 1500);

// Sortie:
// A: 0 (après 0s)
// A: 1 (après 1s)
// A: 2, B: 2 (après 2s) ← B rejoint en cours de route, reçoit la même valeur
```

> [!TIP]
> **Cas où la conversion en Hot est nécessaire** :
> - Vous souhaitez partager une requête HTTP entre plusieurs abonnés
> - Vous souhaitez maintenir une seule connexion WebSocket ou serveur
> - Vous souhaitez utiliser les résultats de calculs coûteux à plusieurs endroits
>
> Pour plus d'informations, voir le chapitre **Subject et Multicast** (Chapitre 5).

## Relation avec les opérateurs pipables

Il n'y a pas d'opérateur pipable correspondant directement aux fonctions de création de base. Elles sont toujours utilisées comme fonctions de création.

Cependant, elles sont utilisées en combinaison avec les opérateurs pipables selon le schéma suivant :

```typescript
import { fromEvent } from 'rxjs';
import { debounceTime, switchMap } from 'rxjs';

// Entrée utilisateur → Attendre 300ms → Appel API
fromEvent(input, 'input').pipe(
  debounceTime(300),
  switchMap(event => fetchSuggestions(event.target.value))
).subscribe(suggestions => console.log(suggestions));
```

## Prochaines étapes

Pour en savoir plus sur le fonctionnement de chaque fonction de création et des exemples pratiques, cliquez sur les liens du tableau ci-dessus.

De plus, en apprenant les [Fonctions de création de combinaison](/fr/guide/creation-functions/combination/), les [Fonctions de création de sélection/partition](/fr/guide/creation-functions/selection/) et les [Fonctions de création conditionnelles](/fr/guide/creation-functions/conditional/), vous pourrez comprendre l'ensemble des fonctions de création.
