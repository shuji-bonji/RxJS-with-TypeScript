---
description: "Explication des 4 types de Subject (Subject, BehaviorSubject, ReplaySubject, AsyncSubject) avec leurs caractéristiques et scénarios d'utilisation. Apprenez à choisir le bon type selon les besoins : présence de valeur initiale, nombre de replays de valeurs, récupération de valeur après complétion, etc., avec des exemples de code TypeScript."
---

# Types de Subject

En plus du `Subject` de base, RxJS fournit plusieurs classes dérivées spécialisées pour des cas d'usage spécifiques. Chacune possède des caractéristiques de comportement différentes, et leur utilisation appropriée permet une programmation réactive plus efficace.

Nous expliquons ici en détail les 4 principaux types de Subject, leurs caractéristiques et leurs scénarios d'utilisation.

## Les 4 types de Subject de base

| Type | Caractéristiques | Cas d'usage principaux |
|------|------|----------------|
| [`Subject`](#subject) | Subject le plus simple<br>Ne reçoit que les valeurs après souscription | Notification d'événements, multicasting |
| [`BehaviorSubject`](#behaviorsubject) | Conserve la dernière valeur et la fournit immédiatement aux nouveaux souscripteurs | Gestion d'état, valeur actuelle des composants UI |
| [`ReplaySubject`](#replaysubject) | Rejoue un nombre spécifié de valeurs passées aux nouveaux souscripteurs | Historique d'opérations, informations de mise à jour récentes |
| [`AsyncSubject`](#asyncsubject) | N'émet que la dernière valeur au moment de la complétion | Résultats de requêtes HTTP/API |

## `Subject` standard {#subject}

[📘 Documentation officielle RxJS : Subject](https://rxjs.dev/api/index/class/Subject)

Le type de Subject le plus simple, qui ne reçoit que les valeurs émises après la souscription.

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

// Pas de valeur initiale, ne reçoit rien au moment de la souscription
subject.subscribe(value => console.log('Observer 1:', value));

subject.next(1);
subject.next(2);

// Deuxième souscription (ne reçoit que les valeurs après souscription)
subject.subscribe(value => console.log('Observer 2:', value));

subject.next(3);
subject.complete();
```

#### Résultat d'exécution
```
Observer 1: 1
Observer 1: 2
Observer 1: 3
Observer 2: 3
```

## BehaviorSubject  {#behaviorsubject}

[📘 Documentation officielle RxJS : BehaviorSubject](https://rxjs.dev/api/index/class/BehaviorSubject)

Nécessite une valeur initiale et conserve toujours la dernière valeur.
Les nouveaux souscripteurs reçoivent immédiatement la dernière valeur au moment de la souscription.

```ts
import { BehaviorSubject } from 'rxjs';

// Créé avec une valeur initiale de 0
const behaviorSubject = new BehaviorSubject<number>(0);

// Reçoit immédiatement la valeur initiale
behaviorSubject.subscribe(value => console.log('Observer 1:', value));

behaviorSubject.next(1);
behaviorSubject.next(2);

// Deuxième souscription (reçoit immédiatement la dernière valeur 2)
behaviorSubject.subscribe(value => console.log('Observer 2:', value));

behaviorSubject.next(3);
behaviorSubject.complete();
```

#### Résultat d'exécution
```
Observer 1: 0
Observer 1: 1
Observer 1: 2
Observer 2: 2
Observer 1: 3
Observer 2: 3
```

### Exemple d'utilisation de BehaviorSubject

#### Gestion de l'état d'authentification utilisateur

```ts
import { BehaviorSubject } from 'rxjs';

interface User {
  id: string;
  name: string;
}

// Valeur initiale null (état non connecté)
const currentUser$ = new BehaviorSubject<User | null>(null);

// Surveillance de l'état de connexion dans les composants, etc.
currentUser$.subscribe(user => {
  if (user) {
    console.log(`Connecté : ${user.name}`);
  } else {
    console.log('État non connecté');
  }
});

// Traitement de connexion
function login(user: User) {
  currentUser$.next(user);
}

// Traitement de déconnexion
function logout() {
  currentUser$.next(null);
}

// Exemple d'utilisation
console.log('Démarrage de l\'application');
// → État non connecté

login({ id: 'user123', name: 'Taro Yamada' });
// → Connecté : Taro Yamada

logout();
// → État non connecté
```

#### Résultat d'exécution
```sh
État non connecté
Démarrage de l'application
Connecté : Taro Yamada
État non connecté
```

## `ReplaySubject` {#replaysubject}
[📘 Documentation officielle RxJS : ReplaySubject](https://rxjs.dev/api/index/class/ReplaySubject)

Mémorise un nombre spécifié de valeurs passées et les retransmet aux nouveaux souscripteurs.
Peut configurer la taille du buffer et la fenêtre temporelle.

```ts
import { ReplaySubject } from 'rxjs';

// Bufferise les 3 dernières valeurs
const replaySubject = new ReplaySubject<number>(3);

replaySubject.next(1);
replaySubject.next(2);
replaySubject.next(3);
replaySubject.next(4);

// Début de souscription (reçoit les 3 dernières valeurs 2,3,4)
replaySubject.subscribe(value => console.log('Observer 1:', value));

replaySubject.next(5);

// Deuxième souscription (reçoit les 3 dernières valeurs 3,4,5)
replaySubject.subscribe(value => console.log('Observer 2:', value));

replaySubject.complete();
```

#### Résultat d'exécution
```
Observer 1: 2
Observer 1: 3
Observer 1: 4
Observer 1: 5
Observer 2: 3
Observer 2: 4
Observer 2: 5
```

### ReplaySubject avec fenêtre temporelle

Il est également possible de bufferiser en fonction du temps.

```ts
import { ReplaySubject } from 'rxjs';

// Bufferise jusqu'à 5 valeurs dans une fenêtre de 500ms
const timeWindowSubject = new ReplaySubject<number>(5, 500);

timeWindowSubject.next(1);

setTimeout(() => {
  timeWindowSubject.next(2);

  // Souscription après 1000ms (ne reçoit pas 1 car au-delà de la fenêtre de 500ms)
  setTimeout(() => {
    timeWindowSubject.subscribe(value => console.log('Réception:', value));
  }, 1000);
}, 100);
```

#### Résultat d'exécution
```
Réception: 2
```

### Exemple d'utilisation de ReplaySubject

#### Gestion de l'historique de recherche récent

```ts
import { ReplaySubject } from 'rxjs';

// Conserve les 5 dernières requêtes de recherche
const searchHistory$ = new ReplaySubject<string>(5);

// Fonction d'exécution de recherche
function search(query: string) {
  console.log(`Exécution de la recherche : ${query}`);
  searchHistory$.next(query);
  // Traitement de recherche réel...
}

// Composant d'affichage de l'historique de recherche
function showSearchHistory() {
  console.log('--- Historique de recherche ---');
  searchHistory$.subscribe(query => {
    console.log(query);
  });
}

// Exemple d'utilisation
search('TypeScript');
search('RxJS');
search('Angular');
search('React');

showSearchHistory();
// Affiche les 5 derniers (dans ce cas 4) historiques de recherche
```

#### Résultat d'exécution
```sh
Exécution de la recherche : TypeScript
Exécution de la recherche : RxJS
Exécution de la recherche : Angular
Exécution de la recherche : React
--- Historique de recherche ---
TypeScript
RxJS
Angular
React
```

## `AsyncSubject` {#asyncsubject}
[📘 Documentation officielle RxJS : AsyncSubject](https://rxjs.dev/api/index/class/AsyncSubject)

Subject qui n'émet que la dernière valeur au moment de la complétion. Les valeurs avant la complétion ne sont pas émises.

```ts
import { AsyncSubject } from 'rxjs';

const asyncSubject = new AsyncSubject<number>();

asyncSubject.subscribe(value => console.log('Observer 1:', value));

asyncSubject.next(1);
asyncSubject.next(2);
asyncSubject.next(3);

// Ne reçoit que la dernière valeur quel que soit le moment de la souscription
asyncSubject.subscribe(value => console.log('Observer 2:', value));

asyncSubject.next(4);
asyncSubject.complete(); // La dernière valeur (4) est émise à la complétion
```

#### Résultat d'exécution
```
Observer 1: 4
Observer 2: 4
```

### Exemple d'utilisation d'AsyncSubject

#### Partage du résultat de requête API

```ts
import { AsyncSubject } from 'rxjs';

interface ApiResponse {
  data: any;
  status: number;
}

function fetchData(url: string) {
  const subject = new AsyncSubject<ApiResponse>();

  // Simulation de requête API
  console.log(`Requête API : ${url}`);
  setTimeout(() => {
    const response = {
      data: { id: 1, name: 'Données d\'exemple' },
      status: 200
    };

    subject.next(response);
    subject.complete();
  }, 1000);

  return subject;
}

// Exemple d'utilisation
const data$ = fetchData('/api/users/1');

// Plusieurs composants peuvent partager le résultat de la même requête
data$.subscribe(response => {
  console.log('Composant 1:', response.data);
});

setTimeout(() => {
  data$.subscribe(response => {
    console.log('Composant 2:', response.data);
  });
}, 1500); // Peut recevoir la valeur même après complétion
```

#### Résultat d'exécution
```sh
Requête API : /api/users/1
Composant 1: {id: 1, name: 'Données d'exemple'}
Composant 2: {id: 1, name: 'Données d'exemple'}
```

## Comparaison et guide de sélection des différents Subject

Points utiles pour choisir le type de Subject approprié.

### Comment choisir un Subject

|type|Critères de sélection|
|---|---|
|`Subject`|Utiliser pour la notification d'événements simple ou la diffusion multicast|
|`BehaviorSubject`|<li>Cas nécessitant toujours une valeur initiale </li><li>Données représentant l'état actuel (état utilisateur, configuration, drapeaux, etc.) </li><li>Valeur actuelle des composants UI</li>|
|`ReplaySubject`|<li>Lorsqu'il faut conserver l'historique des opérations récentes </li><li>Lorsqu'on veut fournir des données passées aux souscripteurs qui arrivent plus tard  </li><li>Flux de données bufferisées</li>|
|`AsyncSubject`|<li>Lorsque seul le résultat final est important (comme les réponses API) </li><li>Lorsqu'on veut partager uniquement la valeur au moment de la complétion, sans les étapes intermédiaires</li>|

### Flux de décision pour la sélection

1. Seule la dernière valeur au moment de la complétion est nécessaire ⇨ `AsyncSubject`
2. Les N dernières valeurs sont nécessaires ⇨ `ReplaySubject`
3. L'état/la valeur actuelle est toujours nécessaire ⇨ `BehaviorSubject`
4. Autres cas (notification d'événements pure, etc.) ⇨ `Subject`

## Patterns d'utilisation dans la conception d'applications

### Exemple de communication inter-modules

```ts
// Service de gestion d'état global de l'application
class AppStateService {
  // Utilisateur actuel (BehaviorSubject car valeur initiale requise)
  private userSubject = new BehaviorSubject<User | null>(null);
  // Publié comme Observable en lecture seule
  readonly user$ = this.userSubject.asObservable();

  // Notifications (Subject pour notification d'événements simple)
  private notificationSubject = new Subject<Notification>();
  readonly notifications$ = this.notificationSubject.asObservable();

  // Recherches récentes (ReplaySubject car historique nécessaire)
  private searchHistorySubject = new ReplaySubject<string>(10);
  readonly searchHistory$ = this.searchHistorySubject.asObservable();

  // Cache des résultats d'appels API (AsyncSubject car seul le résultat final est nécessaire)
  private readonly apiCaches = new Map<string, AsyncSubject<any>>();

  // Exemples de méthodes
  setUser(user: User | null) {
    this.userSubject.next(user);
  }

  notify(notification: Notification) {
    this.notificationSubject.next(notification);
  }

  addSearch(query: string) {
    this.searchHistorySubject.next(query);
  }

  // Cache des résultats API
  fetchData(url: string): Observable<any> {
    if (!this.apiCaches.has(url)) {
      const subject = new AsyncSubject<any>();
      this.apiCaches.set(url, subject);

      // Appel API réel
      fetch(url)
        .then(res => res.json())
        .then(data => {
          subject.next(data);
          subject.complete();
        })
        .catch(err => {
          subject.error(err);
        });
    }

    return this.apiCaches.get(url)!.asObservable();
  }
}
```

## Résumé

Les Subject de RxJS sont des outils puissants pouvant répondre à divers cas d'usage. En comprenant les caractéristiques de chaque type et en les utilisant de manière appropriée, vous pouvez construire des applications réactives efficaces et maintenables.

- `Subject` : Le plus simple, fournit une fonctionnalité de multicasting de base
- `BehaviorSubject` : Conserve toujours l'état actuel et le fournit immédiatement aux nouveaux souscripteurs
- `ReplaySubject` : Conserve l'historique des valeurs récentes et le fournit également aux souscripteurs qui arrivent plus tard
- `AsyncSubject` : N'émet que la valeur finale au moment de la complétion

Le choix du `Subject` approprié dans toutes les situations - gestion d'état, notification d'événements, partage de données - est la clé d'une programmation réactive efficace.
