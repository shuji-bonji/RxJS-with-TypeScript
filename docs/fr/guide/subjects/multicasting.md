---
description: Explication détaillée du mécanisme de multicasting de RxJS. Patterns de base avec Subject, utilisation des opérateurs share et shareReplay, évitement des appels API dupliqués, stratégies de cache, partage d'état d'application et autres patterns de conception pratiques avec exemples de code TypeScript.
---

# Mécanisme de Multicasting

Le multicasting est une technique permettant de diffuser efficacement un flux de données d'un Observable à plusieurs souscripteurs (Observers).
Dans RxJS, cela peut être réalisé via des Subject ou des opérateurs.

## Qu'est-ce que le Multicasting

Un Observable ordinaire (Cold Observable) crée un nouveau flux de données chaque fois qu'il est souscrit. Cela signifie que lorsqu'il y a plusieurs souscripteurs, le même traitement est exécuté plusieurs fois.

L'utilisation du multicasting permet d'exécuter la source de données une seule fois et de distribuer le résultat à plusieurs souscripteurs. Ceci est particulièrement important dans les cas suivants :

- Ne pas vouloir appeler des requêtes HTTP/API en double
- Vouloir exécuter une seule fois des opérations coûteuses (calculs ou effets de bord)
- Partager l'état de l'application entre plusieurs composants

## Patterns de base du Multicasting

### Multicast de base utilisant un Subject

```ts
import { Observable, Subject } from 'rxjs';
import { tap } from 'rxjs';

// Source de données (Cold Observable)
function createDataSource(): Observable<number> {
  return new Observable<number>(observer => {
    console.log('Source de données : connexion');
    // Logique de génération de données (opération coûteuse supposée)
    const id = setInterval(() => {
      const value = Math.round(Math.random() * 100);
      console.log(`Source de données : génération de valeur -> ${value}`);
      observer.next(value);
    }, 1000);

    // Fonction de nettoyage
    return () => {
      console.log('Source de données : déconnexion');
      clearInterval(id);
    };
  });
}

// Implémentation du multicast
function multicast() {
  // Source de données originale
  const source$ = createDataSource().pipe(
    tap(value => console.log(`Traitement source : ${value}`))
  );

  // Subject pour le multicasting
  const subject = new Subject<number>();

  // Connecter la source au Subject
  const subscription = source$.subscribe(subject);

  // Plusieurs souscripteurs s'abonnent au Subject
  console.log('Observer 1 début de souscription');
  const subscription1 = subject.subscribe(value => console.log(`Observer 1: ${value}`));

  // Ajouter un autre souscripteur après 3 secondes
  setTimeout(() => {
    console.log('Observer 2 début de souscription');
    const subscription2 = subject.subscribe(value => console.log(`Observer 2: ${value}`));

    // Terminer toutes les souscriptions après 5 secondes
    setTimeout(() => {
      console.log('Fin de toutes les souscriptions');
      subscription.unsubscribe();
      subscription1.unsubscribe();
      subscription2.unsubscribe();
    }, 5000);
  }, 3000);
}

// Exécution
multicast();
```

#### Résultat d'exécution
```
Source de données : connexion
Observer 1 début de souscription
Source de données : génération de valeur -> 71
Traitement source : 71
Observer 1: 71
Source de données : génération de valeur -> 79
Traitement source : 79
Observer 1: 79
Source de données : génération de valeur -> 63
Traitement source : 63
Observer 1: 63
Observer 2 début de souscription
Source de données : génération de valeur -> 49
Traitement source : 49
Observer 1: 49
Observer 2: 49
Source de données : génération de valeur -> 94
Traitement source : 94
Observer 1: 94
Observer 2: 94
Source de données : génération de valeur -> 89
Traitement source : 89
Observer 1: 89
Observer 2: 89
Source de données : génération de valeur -> 10
Traitement source : 10
Observer 1: 10
Observer 2: 10
Source de données : génération de valeur -> 68
Traitement source : 68
Observer 1: 68
Observer 2: 68
Fin de toutes les souscriptions
Source de données : déconnexion
```

## Opérateurs de Multicast

RxJS fournit des opérateurs dédiés pour implémenter le multicasting.

### Opérateur `share()`
[📘 Documentation officielle RxJS : share()](https://rxjs.dev/api/index/function/share)

L'opérateur le plus simple pour implémenter le multicast.
En interne, il combine `multicast()` et `refCount()`.

```ts
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs';

// Observable qui compte par intervalles
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Source : ${value}`)),
  share() // Activer le multicasting
);

// Premier souscripteur
console.log('Observer 1 début de souscription');
const subscription1 = source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Ajouter un deuxième souscripteur après 2.5 secondes
setTimeout(() => {
  console.log('Observer 2 début de souscription');
  const subscription2 = source$.subscribe(value => console.log(`Observer 2: ${value}`));

  // Désabonner le souscripteur 1 après 5 secondes
  setTimeout(() => {
    console.log('Observer 1 désabonnement');
    subscription1.unsubscribe();
  }, 2500);
}, 2500);
```

#### Résultat d'exécution
```
Observer 1 début de souscription
Source : 0
Observer 1: 0
Observer 2 début de souscription
Source : 1
Observer 1: 1
Observer 2: 1
Source : 2
Observer 1: 2
Observer 2: 2
Source : 3
Observer 1: 3
Observer 2: 3
Observer 1 désabonnement
Source : 4
Observer 2: 4
```

### Contrôle détaillé de `share()`

Au lieu de `refCount()`, depuis RxJS 7, vous pouvez contrôler le comportement plus clairement en passant des options à `share()`.

```ts
import { interval } from 'rxjs';
import { take, share, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(6),
  tap((value) => console.log(`Source : ${value}`)),
  share({
    resetOnError: true,
    resetOnComplete: true,
    resetOnRefCountZero: true,
  })
);

// Premier souscripteur
console.log('Observer 1 début de souscription');
const subscription1 = source$.subscribe((value) =>
  console.log(`Observer 1: ${value}`)
);

// Ajouter un deuxième souscripteur après 2.5 secondes
setTimeout(() => {
  console.log('Observer 2 début de souscription');
  const subscription2 = source$.subscribe((value) =>
    console.log(`Observer 2: ${value}`)
  );

  setTimeout(() => {
    console.log('Observer 1 désabonnement');
    subscription1.unsubscribe();
  }, 1500);
}, 2500);
```

#### Résultat d'exécution
```
Observer 1 début de souscription
Source : 0
Observer 1: 0
Source : 1
Observer 1: 1
Observer 2 début de souscription
Source : 2
Observer 1: 2
Observer 2: 2
Source : 3
Observer 1: 3
Observer 2: 3
Observer 1 désabonnement
Source : 4
Observer 2: 4
Source : 5
Observer 2: 5
```

Cette méthode permet de contrôler clairement le comportement lors de la fin du flux ou lorsque le nombre de souscripteurs atteint zéro.

### Opérateur `shareReplay()`

[📘 Documentation officielle RxJS : shareReplay()](https://rxjs.dev/api/index/function/shareReplay)

Similaire à `share()`, mais mémorise un nombre spécifié de valeurs passées et les fournit également aux souscripteurs qui arrivent plus tard.

```ts
import { interval } from 'rxjs';
import { take, shareReplay, tap } from 'rxjs';

// Utilisation de shareReplay (taille de buffer 2, libéré avec refCount à l'annulation totale des souscriptions)
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Source : ${value}`)),
  shareReplay({ bufferSize: 2, refCount: true }) // Bufferiser les 2 dernières valeurs
);

// Premier souscripteur
console.log('Observer 1 début de souscription');
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Ajouter un deuxième souscripteur après 3.5 secondes
setTimeout(() => {
  console.log('Observer 2 début de souscription - reçoit les 2 dernières valeurs');
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 3500);
```

#### Résultat d'exécution
```
Observer 1 début de souscription
Source : 0
Observer 1: 0
Source : 1
Observer 1: 1
Observer 2 début de souscription - reçoit les 2 dernières valeurs
Observer 2: 0
Observer 2: 1
Source : 2
Observer 1: 2
Observer 2: 2
Source : 3
Observer 1: 3
Observer 2: 3
Source : 4
Observer 1: 4
Observer 2: 4
```

## Timing et Cycle de vie dans le Multicasting

Il est important de comprendre le cycle de vie d'un flux multicast. Particulièrement lors de l'utilisation de l'opérateur `share()`, il faut faire attention aux comportements suivants :

1. Premier souscripteur : `share()` démarre la connexion à l'Observable source lorsque la première souscription a lieu.
2. Tous les souscripteurs se désabonnent : Si la configuration `share({ resetOnRefCountZero: true })` est présente, la connexion à la source est désactivée lorsque le nombre de souscripteurs atteint zéro.
3. Complétion ou erreur : Par défaut, `share()` réinitialise son état interne lorsque complete ou error se produit (si resetOnComplete/resetOnError est true).
4. Réabonnement : Lorsqu'une souscription a lieu après la réinitialisation du flux, il est reconstruit comme un nouveau Observable.

Ainsi, les options de `share()` contrôlent le timing de démarrage, arrêt et régénération du flux en fonction du nombre de souscriptions et de l'état de complétion.

## Cas d'usage pratiques

### Partage de requêtes API

Exemple d'évitement de requêtes dupliquées au même endpoint API.

```ts
import { Observable, of, throwError } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { map, catchError, shareReplay, tap } from 'rxjs';

// Simulation de service API
class UserService {
  private cache = new Map<string, Observable<any>>();

  getUser(id: string): Observable<any> {
    // Retourner du cache si disponible
    if (this.cache.has(id)) {
      console.log(`Récupération de l'utilisateur ID ${id} depuis le cache`);
      return this.cache.get(id)!;
    }

    // Créer une nouvelle requête
    console.log(`Récupération de l'utilisateur ID ${id} depuis l'API`);
    const request$ = ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${id}`).pipe(
      tap(response => console.log('Réponse API:', response)),
      catchError((error: unknown) => {
        console.error('Erreur API:', error);
        // Supprimer du cache
        this.cache.delete(id);
        return throwError(() => new Error('Échec de la récupération de l\'utilisateur'));
      }),
      // Partager avec shareReplay (libéré avec refCount: true à l'annulation totale des souscriptions)
      shareReplay({ bufferSize: 1, refCount: true })
    );

    // Enregistrer dans le cache
    this.cache.set(id, request$);
    return request$;
  }
}

// Exemple d'utilisation
const userService = new UserService();

// Plusieurs composants demandent les mêmes données utilisateur
console.log('Composant 1 : demande de données utilisateur');
userService.getUser('1').subscribe(user => {
  console.log('Composant 1 : réception des données utilisateur', user);
});

// Un autre composant demande les mêmes données un peu plus tard
setTimeout(() => {
  console.log('Composant 2 : demande des mêmes données utilisateur');
  userService.getUser('1').subscribe(user => {
    console.log('Composant 2 : réception des données utilisateur', user);
  });
}, 1000);

// Demande d'un autre utilisateur
setTimeout(() => {
  console.log('Composant 3 : demande de données d\'un autre utilisateur');
  userService.getUser('2').subscribe(user => {
    console.log('Composant 3 : réception des données utilisateur', user);
  });
}, 2000);
```

#### Résultat d'exécution
```
Composant 1 : demande de données utilisateur
Récupération de l'utilisateur ID 1 depuis l'API
Réponse API: {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
Composant 1 : réception des données utilisateur {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
Composant 2 : demande des mêmes données utilisateur
Récupération de l'utilisateur ID 1 depuis le cache
Composant 2 : réception des données utilisateur {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
Composant 3 : demande de données d'un autre utilisateur
Récupération de l'utilisateur ID 2 depuis l'API
Réponse API: {id: 2, name: 'Ervin Howell', username: 'Antonette', email: 'Shanna@melissa.tv', address: {…}, …}
Composant 3 : réception des données utilisateur {id: 2, name: 'Ervin Howell', username: 'Antonette', email: 'Shanna@melissa.tv', address: {…}, …}
```

## Patterns de conception du Multicasting

### Observable Singleton

Pattern de partage d'un observable unique dans toute l'application.

```ts
import { Subject } from 'rxjs';

// Gestion d'état global de l'application
class AppState {
  // Instance singleton
  private static instance: AppState;

  // Flux de notifications global
  private notificationsSubject = new Subject<string>();

  // Observable public (lecture seule)
  readonly notifications$ = this.notificationsSubject.asObservable();

  // Accès singleton
  static getInstance(): AppState {
    if (!AppState.instance) {
      AppState.instance = new AppState();
    }
    return AppState.instance;
  }

  // Méthode pour envoyer une notification
  notify(message: string): void {
    this.notificationsSubject.next(message);
  }
}

// Exemple d'utilisation
const appState = AppState.getInstance();

// Surveiller les notifications (depuis plusieurs composants)
appState.notifications$.subscribe((msg) =>
  console.log('Composant A:', msg)
);
appState.notifications$.subscribe((msg) =>
  console.log('Composant B:', msg)
);

// Envoyer une notification
appState.notify('Mise à jour système disponible');
```

#### Résultat d'exécution
```ts
Composant A: Mise à jour système disponible
Composant B: Mise à jour système disponible
```

## Résumé

Le multicasting est une technique importante pour améliorer l'efficacité et les performances des applications RxJS. Les points principaux sont les suivants :

- Le multicasting permet de partager une source de données unique entre plusieurs souscripteurs
- Peut être implémenté avec des opérateurs comme `share()`, `shareReplay()`, `publish()`
- Peut éviter les requêtes API dupliquées et optimiser les traitements à coût de calcul élevé
- Utile pour la gestion d'état et la communication entre composants

En choisissant une stratégie de multicast appropriée, vous pouvez améliorer la réactivité et l'efficacité de l'application tout en réduisant la quantité de code et en améliorant la maintenabilité.
