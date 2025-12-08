---
description: Subject est une classe spéciale de RxJS qui possède à la fois les propriétés d'Observable et d'Observer. Il permet l'émission et la souscription de données simultanément, et peut diffuser les mêmes valeurs à plusieurs souscripteurs via multicasting. Vous pouvez implémenter des patterns pratiques tels que l'event bus et la gestion d'état tout en préservant la sécurité des types grâce aux paramètres de type TypeScript.
---

# Qu'est-ce qu'un Subject

[📘 Documentation officielle RxJS : Subject](https://rxjs.dev/api/index/class/Subject)

Un Subject est un type spécial d'Observable dans RxJS. Alors qu'un Observable ordinaire fournit un flux de données unidirectionnel, un Subject est une entité hybride qui possède à la fois les propriétés d'un "Observable" et d'un "Observer".

Un Subject possède les caractéristiques suivantes :

- Peut émettre des données (fonctionnalité Observable)
- Peut s'abonner à des données (fonctionnalité Observer)
- Peut délivrer la même valeur à plusieurs souscripteurs (multicasting)
- Ne reçoit que les valeurs émises après la souscription (propriété Hot Observable)


## Utilisation de base d'un Subject

```ts
import { Subject } from 'rxjs';

// Créer un Subject
const subject = new Subject<number>();

// S'abonner en tant qu'Observer
subject.subscribe(value => console.log('Observer A:', value));
subject.subscribe(value => console.log('Observer B:', value));

// Émettre des valeurs en tant qu'Observable
subject.next(1); // Émet la valeur aux deux souscripteurs
subject.next(2); // Émet la valeur aux deux souscripteurs

// Ajouter un nouveau souscripteur (souscription tardive)
subject.subscribe(value => console.log('Observer C:', value));

subject.next(3); // Émet la valeur à tous les souscripteurs

// Notifier la complétion
subject.complete();
```

#### Résultat d'exécution
```
Observer A: 1
Observer B: 1
Observer A: 2
Observer B: 2
Observer A: 3
Observer B: 3
Observer C: 3
```

### Différences avec un Observable ordinaire

Un Subject est un **Hot Observable** et diffère d'un Cold Observable ordinaire sur les points suivants :

- Les données sont émises indépendamment de la présence de souscriptions
- Peut partager les mêmes valeurs entre plusieurs souscripteurs (multicasting)
- Peut émettre des valeurs depuis l'extérieur avec `.next()`
- Ne conserve pas les valeurs passées, ne reçoit que les valeurs après la souscription


## Subject et Multicasting

L'une des fonctions importantes d'un Subject est le "multicasting".
C'est une fonctionnalité qui diffuse efficacement une source de données à plusieurs souscripteurs.

```ts
import { Subject, interval } from 'rxjs';
import { take } from 'rxjs';

// Source de données
const source$ = interval(1000).pipe(take(3));

// Subject pour le multicasting
const subject = new Subject<number>();

// Connecter la source au Subject
source$.subscribe(subject); // Le Subject fonctionne comme un souscripteur

// Plusieurs observateurs s'abonnent au Subject
subject.subscribe(value => console.log('Observer 1:', value));
subject.subscribe(value => console.log('Observer 2:', value));
```

#### Résultat d'exécution
```
Observer 1: 0
Observer 2: 0
Observer 1: 1
Observer 2: 1
Observer 1: 2
Observer 2: 2
```

Ce pattern est également appelé single-source multicast et est utilisé pour diffuser efficacement une source de données unique à plusieurs souscripteurs.


## Deux façons d'utiliser un Subject

Il existe principalement deux façons d'utiliser un Subject. Chacune a des usages et des comportements différents.

### 1. Pattern d'appel manuel de `.next()`

Le Subject est utilisé comme **sujet d'émission de données (Observable)**.
Ce pattern convient aux "envois de valeurs explicites" comme les notifications d'événements ou les mises à jour d'état.

```ts
const subject = new Subject<string>();

subject.subscribe(val => console.log('Observer A:', val));
subject.next('Hello');
subject.next('World');

// Sortie :
// Observer A: Hello
// Observer A: World
```

---

### 2. Pattern de relais d'Observable (Multicasting)

Le Subject joue le rôle de **recevoir des valeurs d'un Observable en tant qu'Observer et de les relayer**.
Cette utilisation est pratique pour **convertir un Cold Observable en Hot et le multicaster**.

```ts
const source$ = interval(1000).pipe(take(3));
const subject = new Subject<number>();

// Observable → Subject (relais)
source$.subscribe(subject);

// Subject → Diffusion vers plusieurs souscripteurs
subject.subscribe(val => console.log('Observer 1:', val));
subject.subscribe(val => console.log('Observer 2:', val));

// Sortie :
// Observer 1: 0
// Observer 2: 0
// Observer 1: 1
// Observer 2: 1
// Observer 1: 2
// Observer 2: 2
```



> [!TIP]
> Lorsque vous appelez `.next()` vous-même, imaginez-le comme "quelqu'un qui parle", et lorsque vous relayez en recevant d'un Observable, imaginez-le comme "quelqu'un qui amplifie les paroles des autres avec un micro" pour faciliter la compréhension.


## Cas d'usage pratiques d'un Subject

Un Subject est particulièrement utile dans les scénarios suivants :

1. **Gestion d'état** - Partager et mettre à jour l'état de l'application
2. **Event bus** - Communication entre composants
3. **Partage de réponses HTTP** - Partager les résultats du même appel API entre plusieurs composants
4. **Gestion centralisée des événements UI** - Traiter diverses opérations UI en un seul endroit

#### Exemple : Implémentation d'un event bus
```ts
import { Subject } from 'rxjs';
import { filter } from 'rxjs';

interface AppEvent {
  type: string;
  payload: any;
}

// Event bus pour toute l'application
const eventBus = new Subject<AppEvent>();

// S'abonner à un type d'événement spécifique
eventBus.pipe(
  filter(event => event.type === 'USER_LOGGED_IN')
).subscribe(event => {
  console.log('Connexion utilisateur:', event.payload);
});

// S'abonner à un autre type d'événement
eventBus.pipe(
  filter(event => event.type === 'DATA_UPDATED')
).subscribe(event => {
  console.log('Mise à jour des données:', event.payload);
});

// Émettre des événements
eventBus.next({ type: 'USER_LOGGED_IN', payload: { userId: '123', username: 'test_user' } });
eventBus.next({ type: 'DATA_UPDATED', payload: { items: [1, 2, 3] } });
```

#### Résultat d'exécution
```
Connexion utilisateur: {userId: '123', username: 'test_user'}
Mise à jour des données: {items: Array(3)}
```

## Résumé

Un Subject est un composant important qui remplit les rôles suivants dans l'écosystème RxJS :

- Possède à la fois les caractéristiques d'Observer (observateur) et d'Observable (observé)
- Fournit un moyen de convertir un Observable cold en hot
- Diffuse efficacement le même flux de données à plusieurs souscripteurs
- Facilite la communication entre composants ou services
- Fournit une base pour la gestion d'état et le traitement d'événements

## 🔗 Sections connexes

- **[Erreurs courantes et solutions](/fr/guide/anti-patterns/common-mistakes#1-subject-の外部公開)** - Meilleures pratiques pour éviter les mauvais usages de Subject
- **[Types de Subject](./types-of-subject)** - BehaviorSubject, ReplaySubject, AsyncSubject, etc.
