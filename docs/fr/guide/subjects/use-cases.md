---
description: Cas d'usage pratiques utilisant la famille Subject (Subject, BehaviorSubject, ReplaySubject, AsyncSubject) pour la gestion d'état, l'event bus, le système de notification global, le cache de données, la gestion de formulaires réactifs, etc., avec de nombreux exemples de code TypeScript. Comprenez les caractéristiques de chaque type de Subject et apprenez à les utiliser dans les situations appropriées.
---

# Cas d'usage des Subject

Les Subject de RxJS peuvent être utilisés dans divers scénarios pratiques. Nous présentons ici des exemples d'utilisation pratique de la famille Subject (Subject, BehaviorSubject, ReplaySubject, AsyncSubject) et expliquons les situations optimales pour chacun.

## Patterns de gestion d'état

### Implémentation d'un store simple

Implémentez un store simple permettant de conserver, mettre à jour et souscrire à l'état de l'application en utilisant `BehaviorSubject`.

```ts
import { BehaviorSubject } from 'rxjs';
import { map } from 'rxjs';

interface AppState {
  user: { name: string; role: string } | null;
  theme: 'light' | 'dark';
  notifications: string[];
}

// État initial
const initialState: AppState = {
  user: null,
  theme: 'light',
  notifications: []
};

class Store {
  // Gérer l'état avec BehaviorSubject
  private state$ = new BehaviorSubject<AppState>(initialState);

  // Méthode de lecture de l'état
  getState() {
    return this.state$.getValue();
  }

  // Obtenir une propriété spécifiée comme Observable
  select<K extends keyof AppState>(key: K) {
    return this.state$.pipe(
      map(state => state[key])
    );
  }

  // Mise à jour de l'état
  setState(newState: Partial<AppState>) {
    this.state$.next({
      ...this.getState(),
      ...newState
    });
  }

  // Publier l'état comme Observable
  get state() {
    return this.state$.asObservable();
  }
}

// Exemple d'utilisation
const store = new Store();

// Surveiller l'état
store.select('user').subscribe(user => {
  console.log('Changement d\'état utilisateur:', user?.name, user?.role);
});

// Surveiller le changement de thème
store.select('theme').subscribe(theme => {
  console.log('Changement de thème:', theme);
  document.body.className = theme; // Refléter dans l'UI
});

// Mise à jour de l'état
store.setState({ user: { name: 'Taro Yamada', role: 'admin' } });
store.setState({ theme: 'dark' });
```

#### Résultat d'exécution
```sh
Changement d'état utilisateur: undefined undefined
Changement de thème: light
Changement d'état utilisateur: Taro Yamada admin
Changement de thème: light
Changement d'état utilisateur: Taro Yamada admin
Changement de thème: dark
```

Ce pattern est pratique pour les petites applications ou lorsque vous n'utilisez pas de bibliothèque de gestion d'état à grande échelle comme NgRx ou Redux.

## Communication inter-composants

### Implémentation d'un event bus

Implémentez un event bus basé sur `Subject` pouvant gérer différents types de données selon le type de notification, et effectuez une communication inter-composants.

```ts
import { Subject } from 'rxjs';
import { filter, map } from 'rxjs';

type EventPayloadMap = {
  USER_LOGIN: { username: string; timestamp: number };
  DATA_UPDATED: any;
  NOTIFICATION: string;
};

// Définition du type d'événement
type EventType = keyof EventPayloadMap;

interface AppEvent<K extends EventType> {
  type: K;
  payload: EventPayloadMap[K];
}

// Service d'event bus
class EventBusService {
  private eventSubject = new Subject<AppEvent<unknown>>();

  emit<K extends EventType>(type: K, payload: EventPayloadMap[K]): void {
    this.eventSubject.next({ type, payload });
  }

  // S'abonner à un type d'événement spécifique
  on<K extends EventType>(type: K) {
    return this.eventSubject.pipe(
      filter((event): event is AppEvent<K> => event.type === type),
      map((event) => event.payload)
    );
  }
}
// Exemple d'utilisation) Communication inter-composants
const eventBus = new EventBusService();

// Composant d'en-tête (affichage des notifications)
eventBus.on('NOTIFICATION').subscribe((message) => {
  console.log('En-tête : affichage de la notification:', message);
});

// Composant utilisateur (surveillance de l'état de connexion)
eventBus.on('USER_LOGIN').subscribe((user) => {
  console.log('Composant utilisateur : connexion détectée:', user.username);
});

// Composant de paramètres (surveillance des mises à jour de données)
eventBus.on('DATA_UPDATED').subscribe((data) => {
  console.log('Composant de paramètres : mise à jour des données:', data);
});

// Émission d'événements
eventBus.emit('USER_LOGIN', { username: 'user123', timestamp: Date.now() });
eventBus.emit('NOTIFICATION', 'Vous avez de nouveaux messages');
```

#### Résultat d'exécution
```sh
Composant utilisateur : connexion détectée: user123
En-tête : affichage de la notification: Vous avez de nouveaux messages
```

Le pattern d'event bus est une excellente méthode pour réaliser une communication inter-composants faiblement couplée. Il convient particulièrement à la communication entre composants hiérarchiquement éloignés.

> [!CAUTION]
> 💡 Dans les applications réelles, ne pas effectuer de désabonnement (`unsubscribe()`) peut entraîner des fuites mémoire. Envisagez également un traitement de désabonnement utilisant `takeUntil()`, etc.

## Mise en cache de données API

### Partage et cache des résultats de requête

Réalisez le partage et la mise en cache de données émises une seule fois, comme les requêtes HTTP, en utilisant `AsyncSubject`.

```ts
import { Observable, AsyncSubject, of, throwError } from 'rxjs';
import { tap, catchError, delay } from 'rxjs';

class ApiCacheService {
  private cache = new Map<string, AsyncSubject<any>>();

  fetchData<T>(url: string): Observable<T> {
    // Retourner du cache si existant
    if (this.cache.has(url)) {
      console.log(`Récupération des données depuis le cache : ${url}`);
      return this.cache.get(url)!.asObservable() as Observable<T>;
    }

    // Créer une nouvelle requête si pas de cache
    console.log(`Exécution de la requête API : ${url}`);
    const subject = new AsyncSubject<T>();
    this.cache.set(url, subject);

    // Simulation de requête API
    this.makeRequest<T>(url)
      .pipe(
        tap((data) => {
          subject.next(data);
          subject.complete();
        }),
        catchError((error: unknown) => {
          // Supprimer du cache en cas d'erreur
          this.cache.delete(url);
          subject.error(error);
          return throwError(() => error);
        })
      )
      .subscribe();

    return subject.asObservable();
  }

  // Traitement de requête API réelle
  private makeRequest<T>(url: string): Observable<T> {
    // Dans une application réelle, utiliser fetch ou un client HTTP
    return of({
      data: 'Données d\'exemple',
      timestamp: Date.now(),
    } as unknown as T).pipe(
      tap(() => console.log('Réception de la réponse API')),
      // Simuler un délai aléatoire
      delay(Math.random() * 1000 + 500)
    );
  }

  // Nettoyer le cache
  clearCache(url?: string): void {
    if (url) {
      this.cache.delete(url);
    } else {
      this.cache.clear();
    }
    console.log('Cache nettoyé');
  }
}

// Exemple d'utilisation
const apiCache = new ApiCacheService();

// Plusieurs composants demandent les mêmes données API
apiCache.fetchData('/api/products').subscribe((data) => {
  console.log('Composant 1 : réception des données', data);
});

// Un peu plus tard, un autre composant demande les mêmes données (récupération depuis le cache)
setTimeout(() => {
  apiCache.fetchData('/api/products').subscribe((data) => {
    console.log('Composant 2 : réception des données', data);
  });
}, 1000);

// Nouvelle requête après nettoyage du cache
setTimeout(() => {
  apiCache.clearCache();
  apiCache.fetchData('/api/products').subscribe((data) => {
    console.log('Composant 3 : réception des données (après nettoyage du cache)', data);
  });
}, 2000);
```

#### Résultat d'exécution
```sh
Exécution de la requête API : /api/products
Réception de la réponse API
Composant 1 : réception des données {data: 'Données d'exemple', timestamp: 1745405703582}
Récupération des données depuis le cache : /api/products
Composant 2 : réception des données {data: 'Données d'exemple', timestamp: 1745405703582}
Cache nettoyé
Exécution de la requête API : /api/products
Réception de la réponse API
Composant 3 : réception des données (après nettoyage du cache) {data: 'Données d'exemple', timestamp: 1745405705585}
```

Ce pattern utilisant AsyncSubject est optimal pour les requêtes API où seule la dernière valeur au moment de la complétion est importante. Il empêche également l'émission dupliquée de la même requête.

> [!TIP]
> 💡 Si `error()` est appelé sur `AsyncSubject`, la valeur n'est pas émise et seul `error` est notifié, soyez attentif.


## Gestion de formulaires

Gérez la valeur actuelle et l'état de validation d'un formulaire réactif en utilisant `BehaviorSubject`.
### Liaison bidirectionnelle de la valeur de formulaire

```ts
import { BehaviorSubject } from 'rxjs';
import { debounceTime, distinctUntilChanged } from 'rxjs';

interface UserForm {
  name: string;
  email: string;
  age: number;
}

class ReactiveForm {
  // BehaviorSubject avec valeur initiale
  private formSubject = new BehaviorSubject<UserForm>({
    name: '',
    email: '',
    age: 0
  });

  // Observable public
  formValues$ = this.formSubject.asObservable();

  // Résultat de validation
  private validSubject = new BehaviorSubject<boolean>(false);
  valid$ = this.validSubject.asObservable();

  constructor() {
    // Exécuter la validation lors du changement de valeur
    this.formValues$.pipe(
      debounceTime(300),
      distinctUntilChanged((prev, curr) => JSON.stringify(prev) === JSON.stringify(curr))
    ).subscribe(form => {
      this.validateForm(form);
    });
  }

  // Mise à jour de la valeur du champ
  updateField<K extends keyof UserForm>(field: K, value: UserForm[K]) {
    const currentForm = this.formSubject.getValue();
    this.formSubject.next({
      ...currentForm,
      [field]: value
    });
  }

  // Récupération du formulaire
  getForm(): UserForm {
    return this.formSubject.getValue();
  }

  // Validation
  private validateForm(form: UserForm) {
    const isValid =
      form.name.length > 0 &&
      form.email.includes('@') &&
      form.age > 0;

    this.validSubject.next(isValid);
  }

  // Soumission du formulaire
  submit() {
    if (this.validSubject.getValue()) {
      console.log('Soumission du formulaire:', this.getForm());
      // Requête API, etc.
    } else {
      console.error('Le formulaire est invalide');
    }
  }
}

// Exemple d'utilisation
const form = new ReactiveForm();

// Surveiller les valeurs du formulaire
form.formValues$.subscribe(values => {
  console.log('Changement de valeur du formulaire:', values);
  // Traitement de mise à jour de l'UI, etc.
});

// Surveiller l'état de validation
form.valid$.subscribe(isValid => {
  console.log('Validité du formulaire:', isValid);
  // Activer/désactiver le bouton de soumission, etc.
});

// Simuler la saisie utilisateur
form.updateField('name', 'Taro Yamada');
form.updateField('email', 'yamada@example.com');
form.updateField('age', 30);

// Soumission du formulaire
form.submit();
```

#### Résultat d'exécution
```sh
Changement de valeur du formulaire: {name: '', email: '', age: 0}
Validité du formulaire: false
Changement de valeur du formulaire: {name: 'Taro Yamada', email: '', age: 0}
Changement de valeur du formulaire: {name: 'Taro Yamada', email: 'yamada@example.com', age: 0}
Changement de valeur du formulaire: {name: 'Taro Yamada', email: 'yamada@example.com', age: 30}
Le formulaire est invalide
submit @
(anonyme) @ Analyser cette erreur
Validité du formulaire: true
```


Ce pattern est particulièrement utile pour l'implémentation de formulaires réactifs. BehaviorSubject est optimal pour la gestion de l'état de formulaire car il conserve toujours la valeur actuelle.

## Logging et historique

Construisez un mécanisme de gestion de logs pouvant conserver et réafficher l'historique des opérations passées en utilisant `ReplaySubject`.
### Gestion de l'historique des opérations

```ts
import { Observable, ReplaySubject } from 'rxjs';
import { tap } from 'rxjs';

interface LogEntry {
  action: string;
  timestamp: number;
  data?: any;
}

class ActivityLogger {
  // Conserver les 10 dernières entrées de log
  private logSubject = new ReplaySubject<LogEntry>(10);
  logs$ = this.logSubject.asObservable();

  // Ajouter une entrée de log
  log(action: string, data?: any) {
    const entry: LogEntry = {
      action,
      timestamp: Date.now(),
      data
    };

    this.logSubject.next(entry);
    console.log(`Enregistrement du log : ${action}`, data);
  }

  // Envelopper un autre Observable et enregistrer le log
  wrapWithLogging<T>(source$: Observable<T>, actionName: string): Observable<T> {
    return source$.pipe(
      tap(data => this.log(actionName, data))
    );
  }
}

// Exemple d'utilisation
const logger = new ActivityLogger();

// Surveiller les logs (pour affichage dans l'UI, etc.)
logger.logs$.subscribe(log => {
  const time = new Date(log.timestamp).toLocaleTimeString();
  console.log(`[${time}] ${log.action}`);
});

// Enregistrer diverses opérations dans le log
logger.log('Démarrage de l\'application');
logger.log('Connexion utilisateur', { userId: 'user123' });

// Un peu plus tard, un nouveau composant s'abonne en incluant les logs passés
setTimeout(() => {
  console.log('--- Le visualiseur d\'historique affiche les logs passés inclus ---');
  logger.logs$.subscribe(log => {
    const time = new Date(log.timestamp).toLocaleTimeString();
    console.log(`Historique : [${time}] ${log.action}`);
  });

  // Ajouter encore un log
  logger.log('Mise à jour des données', { itemId: 456 });
}, 1000);
```
#### Résultat d'exécution
```sh
[19:58:40] Démarrage de l'application
Enregistrement du log : Démarrage de l'application undefined
[19:58:40] Connexion utilisateur
Enregistrement du log : Connexion utilisateur {userId: 'user123'}
--- Le visualiseur d'historique affiche les logs passés inclus ---
Historique : [19:58:40] Démarrage de l'application
Historique : [19:58:40] Connexion utilisateur
[19:58:41] Mise à jour des données
Historique : [19:58:41] Mise à jour des données
Enregistrement du log : Mise à jour des données {itemId: 456}
```

L'utilisation de ReplaySubject est optimale pour la gestion d'historique car elle peut fournir les entrées de log passées aux nouveaux souscripteurs. Utile pour le suivi des opérations utilisateur et la collecte d'informations de débogage.

> [!IMPORTANT]
> ⚠️ Si vous ne spécifiez pas de taille de buffer pour `ReplaySubject`, toutes les valeurs continueront d'être conservées en mémoire, soyez donc prudent avec les grandes quantités de données ou les applications fonctionnant longtemps.

## Gestion des traitements asynchrones

Gérez en temps réel l'état de progression et l'état actif de plusieurs tâches en utilisant `Subject` et `BehaviorSubject`.
### Gestion de la progression de tâches de longue durée

```ts
import { Subject, BehaviorSubject } from 'rxjs';

interface TaskProgress {
  taskId: string;
  progress: number; // 0-100
  status: 'pending' | 'running' | 'completed' | 'error';
  message?: string;
}

class TaskManager {
  // Notification de la progression des tâches
  private progressSubject = new Subject<TaskProgress>();
  progress$ = this.progressSubject.asObservable();

  // Tâches actuellement en cours d'exécution
  private activeTasksSubject = new BehaviorSubject<string[]>([]);
  activeTasks$ = this.activeTasksSubject.asObservable();

  // Démarrer une tâche
  startTask(taskId: string, taskFn: (update: (progress: number) => void) => Promise<any>) {
    // Ajouter à la liste des tâches actives
    const currentTasks = this.activeTasksSubject.getValue();
    this.activeTasksSubject.next([...currentTasks, taskId]);

    // Notification de progression initiale
    this.progressSubject.next({
      taskId,
      progress: 0,
      status: 'running'
    });

    // Fonction de mise à jour de la progression
    const updateProgress = (progress: number) => {
      this.progressSubject.next({
        taskId,
        progress,
        status: 'running'
      });
    };

    // Exécution de la tâche
    return taskFn(updateProgress)
      .then(result => {
        // Notification de complétion
        this.progressSubject.next({
          taskId,
          progress: 100,
          status: 'completed'
        });
        return result;
      })
      .catch(error => {
        // Notification d'erreur
        this.progressSubject.next({
          taskId,
          progress: 0,
          status: 'error',
          message: error.message
        });
        throw error;
      })
      .finally(() => {
        // Supprimer de la liste des tâches actives
        const tasks = this.activeTasksSubject.getValue();
        this.activeTasksSubject.next(tasks.filter(id => id !== taskId));
      });
  }
}

// Exemple d'utilisation
const taskManager = new TaskManager();

// Afficher la progression dans une barre de progression UI, etc.
taskManager.progress$.subscribe(progress => {
  console.log(`Tâche ${progress.taskId} : ${progress.progress}% - ${progress.status}`);

  // Code de mise à jour de l'UI
  // progressBar.setValue(progress.progress);
  // statusLabel.setText(progress.status);
});

// Afficher le nombre de tâches actives
taskManager.activeTasks$.subscribe(tasks => {
  console.log(`Nombre de tâches en cours : ${tasks.length}`);
});

// Simulation d'une tâche de longue durée
taskManager.startTask('file-upload', (update) => {
  return new Promise((resolve) => {
    let progress = 0;

    // Simulation de progression
    const interval = setInterval(() => {
      progress += 10;
      update(progress);

      if (progress >= 100) {
        clearInterval(interval);
        resolve('Upload terminé');
      }
    }, 500);
  });
});
```

#### Résultat d'exécution
```sh
Nombre de tâches en cours : 0
Nombre de tâches en cours : 1
Tâche file-upload : 0% - running
Tâche file-upload : 10% - running
Tâche file-upload : 20% - running
Tâche file-upload : 30% - running
Tâche file-upload : 40% - running
Tâche file-upload : 50% - running
Tâche file-upload : 60% - running
Tâche file-upload : 70% - running
Tâche file-upload : 80% - running
Tâche file-upload : 90% - running
Tâche file-upload : 100% - running
Tâche file-upload : 100% - completed
Nombre de tâches en cours : 0
```

Ce pattern utilise Subject pour notifier en temps réel l'état de progression des tâches de longue durée. Convient pour l'affichage de la progression d'upload de fichiers, traitement de données, opérations en arrière-plan, etc.

## Mises à jour en temps réel

Gérez l'état de connexion WebSocket, les messages reçus et le contrôle de reconnexion en utilisant plusieurs Subject.
### Gestion de flux WebSocket

```ts
import { Subject, BehaviorSubject, timer, Observable } from 'rxjs';
import { takeUntil, filter, map } from 'rxjs';

interface WebSocketMessage {
  type: string;
  data: any;
}

class WebSocketService {
  private socket: WebSocket | null = null;
  private url: string;

  // État de connexion
  private connectionStatusSubject = new BehaviorSubject<boolean>(false);
  connectionStatus$ = this.connectionStatusSubject.asObservable();

  // Flux de messages
  private messagesSubject = new Subject<WebSocketMessage>();
  messages$ = this.messagesSubject.asObservable();

  // Subject pour la fin de connexion
  private destroySubject = new Subject<void>();

  constructor(url: string) {
    this.url = url;
  }

  // Démarrer la connexion WebSocket
  connect(): void {
    if (this.socket) {
      return; // Déjà connecté
    }

    this.socket = new WebSocket(this.url);

    // Configuration des gestionnaires d'événements
    this.socket.addEventListener('open', () => {
      console.log('Connexion WebSocket établie');
      this.connectionStatusSubject.next(true);
    });

    this.socket.addEventListener('message', (event) => {
      try {
        const message = JSON.parse(event.data) as WebSocketMessage;
        this.messagesSubject.next(message);
      } catch (e) {
        console.error('Erreur d\'analyse du message:', e);
      }
    });

    this.socket.addEventListener('close', () => {
      console.log('Connexion WebSocket terminée');
      this.connectionStatusSubject.next(false);
      this.socket = null;

      // Reconnexion automatique
      this.reconnect();
    });

    this.socket.addEventListener('error', (error) => {
      console.error('Erreur WebSocket:', error);
      this.connectionStatusSubject.next(false);
    });
  }

  // Logique de reconnexion
  private reconnect(): void {
    // Se reconnecter si destroy n'a pas été appelé
    timer(3000)
      .pipe(takeUntil(this.destroySubject))
      .subscribe(() => {
        console.log('Tentative de reconnexion WebSocket...');
        this.connect();
      });
  }

  // Envoyer un message
  send(type: string, data: any): void {
    if (this.socket && this.socket.readyState === WebSocket.OPEN) {
      const message: WebSocketMessage = { type, data };
      this.socket.send(JSON.stringify(message));
    } else {
      console.error('WebSocket non connecté');
    }
  }

  // Obtenir uniquement les messages d'un type spécifique
  getMessagesOfType<T>(type: string): Observable<T> {
    return this.messages$.pipe(
      filter((msg) => msg.type === type),
      map((msg) => msg.data as T)
    );
  }

  // Déconnecter
  disconnect(): void {
    this.destroySubject.next();
    this.destroySubject.complete();

    if (this.socket) {
      this.socket.close();
      this.socket = null;
    }
  }
}

// Exemple d'utilisation
const wsService = new WebSocketService('wss://echo.websocket.org');

// Surveiller l'état de connexion
wsService.connectionStatus$.subscribe((isConnected) => {
  console.log('État de connexion:', isConnected ? 'En ligne' : 'Hors ligne');
  // Mise à jour de l'UI, etc.
});

// Surveiller tous les messages
wsService.messages$.subscribe((message) => {
  console.log('Message reçu:', message);
});

// Surveiller uniquement les messages d'un type spécifique
wsService
  .getMessagesOfType<{ price: number }>('stock-update')
  .subscribe((stockData) => {
    console.log(`Mise à jour du cours : ${stockData.price}`);
  });

// Démarrer la connexion
wsService.connect();

// Envoyer un message
setTimeout(() => {
  wsService.send('chat-message', { text: 'Bonjour !' });
}, 1000);

// À la fin de l'application
// wsService.disconnect();
```

#### Résultat d'exécution
```sh
État de connexion: Hors ligne
Connexion WebSocket établie
État de connexion: En ligne
Erreur d'analyse du message: SyntaxError: Unexpected token 'R', "Request se"... is not valid JSON
  at JSON.parse (<anonymous>)
  at WebSocket.<anonymous> (:30)
(anonyme) @ Analyser cette erreur
Message reçu: {type: 'chat-message', data: {…}}
```

Ce pattern de gestion WebSocket est optimal pour les applications nécessitant une communication en temps réel. Utilisez Subject pour gérer l'état de connexion et le flux de messages, et partagez entre plusieurs composants.

## Directives pour le choix de Subject

| Cas d'usage | Subject recommandé | Explication |
|--------------|-------------|------|
| Notification d'événements/communication | `Subject` | Convient pour la communication unidirectionnelle simple |
| Conservation de valeur actuelle/gestion d'état | `BehaviorSubject` | Nécessite une valeur initiale, dernière valeur toujours accessible |
| Flux avec historique/logs | `ReplaySubject` | Peut fournir également les valeurs passées aux souscripteurs |
| Fourniture groupée de valeur finale/partage de réponse | `AsyncSubject` | Notifie uniquement la dernière valeur au moment de la complétion |

> 💡 Ajouter `$` à la fin du nom de variable est une convention de nommage courante dans RxJS pour indiquer qu'il s'agit d'un Observable.

## Résumé

La famille Subject de RxJS est un outil puissant répondant à divers cas d'usage tels que :

- **BehaviorSubject** : Gestion d'état, gestion de formulaires, affichage de valeur actuelle
- **Subject** : Notification d'événements, communication inter-composants
- **ReplaySubject** : Gestion d'historique, logs d'opérations, composants avec participation tardive
- **AsyncSubject** : Cache de réponse API, partage de résultats de calcul

En combinant ces patterns de manière appropriée, vous pouvez construire des applications réactives et maintenables. Faites particulièrement attention au désabonnement au bon moment pour éviter les fuites mémoire.
