---
description: "Décrit comment mettre en œuvre la multidiffusion en utilisant l'opérateur share(). Partage le même Observable avec plusieurs abonnés pour réduire le traitement en double (appels API, calculs). Montre comment il diffère de shareReplay(), la conversion Cold/Hot et l'implémentation type-safe en TypeScript."
---

# share - Partager un Observable avec plusieurs abonnés

L'opérateur `share()` est l'opérateur le plus simple pour implémenter le multicasting dans RxJS.
En permettant à plusieurs abonnés de partager la même source de données, le traitement en double (requêtes API, calculs, etc.) peut être réduit.

[📘 Documentation officielle RxJS - `share()`](https://rxjs.dev/api/index/function/share)

## 🔰 Utilisation de base

```typescript
import { interval, share, take, tap } from 'rxjs';

// Observable qui compte par intervalles
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Source : ${value}`)),
  share() // Activer la multidiffusion
);

// Premier abonné
console.log('Observer 1 abonnement démarré');
const subscription1 = source$.subscribe(value =>
  console.log(`Observer 1 : ${value}`)
);

// Ajout d'un deuxième abonné après 2,5 secondes
setTimeout(() => {
  console.log('Observer 2 abonnement démarré');
  source$.subscribe(value =>
    console.log(`Observer 2 : ${value}`)
  );

  // Désabonnement de l'abonné 1 après 2,5 secondes
  setTimeout(() => {
    console.log('Observer 1 désabonnement');
    subscription1.unsubscribe();
  }, 2500);
}, 2500);
```

### Résultat de l'exécution

```
Observer 1 abonnement démarré
Source : 0
Observer 1 : 0
Source : 1
Observer 1 : 1
Observer 2 abonnement démarré
Source : 2
Observer 1 : 2
Observer 2 : 2
Source : 3
Observer 1 : 3
Observer 2 : 3
Observer 1 désabonnement
Source : 4
Observer 2 : 4
```

**Points importants** :
- Le processus source (`tap`) n'est exécuté qu'une seule fois
- Tous les abonnés reçoivent la même valeur
- Les abonnés qui rejoignent en cours de route ne reçoivent que les valeurs après leur inscription

## 💡 Comment fonctionne share()

`share()` est un opérateur de multidiffusion RxJS standard. En interne, il utilise Subject pour diffuser à plusieurs abonnés.

> [!NOTE]
> **Changé depuis RxJS v7** : précédemment décrit comme une combinaison de `multicast()` et `refCount()`, ces opérateurs ont été dépréciés dans la v7 et supprimés dans la v8. Actuellement, `share()` est la méthode standard de multidiffusion. Pour plus d'informations, voir [Documentation officielle RxJS - Multicasting](https://rxjs.dev/deprecations/multicasting).

**Flux d'opération** :
- **Au premier abonnement** : initie une connexion à l'Observable source et crée un Subject interne
- **Abonnés ajoutés** : partagent la connexion existante (diffusion des valeurs par le biais du Subject)
- **Tous les abonnés se désabonnent** : déconnexion de la source (si `resetOnRefCountZero: true`)
- **Réabonnement** : démarrage d'une nouvelle connexion (selon le paramètre de réinitialisation)

## 🎯 Options de contrôle avancées (RxJS 7+)

Dans RxJS 7+, le comportement peut être finement contrôlé en passant des options à `share()`.

```typescript
import { interval, share, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(6),
  tap((value) => console.log(`Source : ${value}`)),
  share({
    resetOnError: true,       // Réinitialiser l'état interne en cas d'erreur
    resetOnComplete: true,     // Réinitialiser l'état interne à la fin du flux
    resetOnRefCountZero: true, // Déconnexion quand le nombre d'abonnés atteint zéro
  })
);
```

### Les options en détail

| Option | Valeur par défaut | Description |
|-----------|----------|------|
| `resetOnError` | `true` | Réinitialiser l'état interne en cas d'erreur |
| `resetOnComplete` | `true` | Réinitialiser l'état interne à la fin du flux |
| `resetOnRefCountZero` | `true` | Déconnexion quand le nombre d'abonnés atteint zéro |
| `connector` | `() => new Subject()` | Spécifier un Subject personnalisé |

### Contrôle avancé avec l'option connector

L'option `connector` peut être utilisée pour obtenir un comportement équivalent à celui de `shareReplay`.

```typescript
import { interval, ReplaySubject } from 'rxjs';
import { take, share, tap } from 'rxjs';

// Mettre en tampon le dernier élément en utilisant ReplaySubject
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Source : ${value}`)),
  share({
    connector: () => new ReplaySubject(1),
    resetOnError: false,
    resetOnComplete: false,
    resetOnRefCountZero: false
  })
);

// Premier abonné
source$.subscribe(value => console.log(`Observer 1 : ${value}`));

// S'abonner après 2,5 secondes (reçoit le dernier élément)
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2 : ${value}`));
}, 2500);
```

**Résultat de l'exécution** :
```
Source : 0
Observer 1 : 0
Source : 1
Observer 1 : 1
Observer 2 : 1  // ← Reçoit la dernière valeur même en rejoignant en cours de route
Source : 2
Observer 1 : 2
Observer 2 : 2
...
```

> [!TIP]
> Cette méthode peut être utilisée comme alternative à `shareReplay(1)`. En définissant `resetOnRefCountZero: false`, la connexion est maintenue même si le nombre de références tombe à zéro, ce qui évite le problème du « cache persistant » de `shareReplay`.

## 📊 Comparaison sans share()

### ❌ Sans share() (Cold Observable)

```typescript
import { interval, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Source : ${value}`))
);

// Abonné 1
source$.subscribe(value => console.log(`Observer 1 : ${value}`));

// Abonné 2
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2 : ${value}`));
}, 1500);
```

**Résultat de l'exécution** :
```
Source : 0
Observer 1 : 0
Source : 1
Observer 1 : 1
Source : 0    // ← Un nouveau flux est démarré
Observer 2 : 0
Source : 2
Observer 1 : 2
Source : 1
Observer 2 : 1
Source : 2
Observer 2 : 2
```

Chaque abonné dispose d'un flux indépendant et le processus source est exécuté en double.

### ✅ Avec share() (Hot Observable)

```typescript
import { interval, share, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Source : ${value}`)),
  share()
);

// Abonné 1
source$.subscribe(value => console.log(`Observer 1 : ${value}`));

// Abonné 2
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2 : ${value}`));
}, 1500);
```

**Résultat de l'exécution** :
```
Source : 0
Observer 1 : 0
Source : 1
Observer 1 : 1
Observer 2 : 1  // ← Partage le même flux
Source : 2
Observer 1 : 2
Observer 2 : 2
```

## 💼 Cas d'utilisation pratiques

### Prévenir les requêtes API en double

```typescript
import { ajax } from 'rxjs/ajax';
import { share, tap } from 'rxjs';

// Observable pour récupérer les informations utilisateur
const getUser$ = ajax.getJSON('https://jsonplaceholder.typicode.com/users/1').pipe(
  tap(() => console.log('Requête API exécutée')),
  share() // Empêcher les requêtes en double dans plusieurs composants
);

// Composant 1
getUser$.subscribe(user => console.log('Composant 1 :', user));

// Composant 2 (requête presque simultanée)
getUser$.subscribe(user => console.log('Composant 2 :', user));

// Résultat : la requête API n'est exécutée qu'une seule fois
```

### Partage de l'acquisition périodique de données

```typescript
import { timer, share, switchMap, tap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Liste TODO récupérée toutes les 5 secondes (requête API partagée)
const sharedTodos$ = timer(0, 5000).pipe(
  tap(() => console.log('Requête API exécutée')),
  switchMap(() => ajax.getJSON('https://jsonplaceholder.typicode.com/todos?_limit=3')),
  share() // Requête API partagée par plusieurs abonnés
);

// Plusieurs composants utilisent le même flux de données
sharedTodos$.subscribe(todos => console.log('Composant A :', todos));
sharedTodos$.subscribe(todos => console.log('Composant B :', todos));

// Résultat : une seule requête API est exécutée toutes les 5 secondes, les deux composants reçoivent les mêmes données
```

## ⚠️ Notes

1. **Note sur le timing** : les abonnés qui rejoignent en cours de route ne recevront pas les valeurs précédentes
2. **Propagation des erreurs** : si une erreur se produit, elle affecte tous les abonnés
3. **Gestion de la mémoire** : le fait de ne pas se désabonner correctement peut entraîner des fuites de mémoire

## 🔄 Opérateurs connexes

- **[shareReplay()](/fr/guide/operators/multicasting/shareReplay)** - met en tampon les valeurs passées pour les abonnés suivants
- **[Subject](/fr/guide/subjects/what-is-subject)** - la classe sous-jacente pour la multidiffusion

> [!WARNING]
> **Opérateurs dépréciés** : les anciennes API de multidiffusion telles que `publish()`, `multicast()` et `refCount()` ont été dépréciées dans RxJS v7 et supprimées dans la v8. Utilisez `share()` ou `connectable()`/`connect()` à la place.

## Résumé

L'opérateur `share()` permet de :
- Partager le même Observable avec plusieurs abonnés
- Éviter l'exécution en double des requêtes API et les traitements lourds
- Utiliser facilement les bases du multicasting
- Utiliser des options de contrôle fin disponibles dans RxJS 7+

Lorsque plusieurs composants ont besoin de la même source de données, `share()` peut améliorer les performances de manière significative.
