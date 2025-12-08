---
description: "Explique les cas d'utilisation pratiques des opérateurs de combinaison RxJS (combineLatest, forkJoin, merge, concat, withLatestFrom, etc.). Présente des patterns pratiques pour combiner plusieurs Observables comme la validation de formulaire et l'intégration API, l'exécution parallèle de multiples requêtes, la synchronisation de données temps réel, le traitement séquentiel de flux avec des exemples de code TypeScript."
---

# Cas d'utilisation pratiques

Ce chapitre présente des **cas d'utilisation pratiques** utilisant les opérateurs de combinaison RxJS.
Approfondissez votre compréhension à travers des scénarios tels que les opérations UI et la communication API, utiles pour le développement d'applications réelles.

## Validation de formulaire et requêtes API

Un exemple utilisant `combineLatest` pour valider plusieurs champs de formulaire.

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, debounceTime, startWith } from 'rxjs';

// Création de l'UI du formulaire
const formContainer = document.createElement('div');
formContainer.innerHTML = '<h3>Formulaire d\'inscription:</h3>';
document.body.appendChild(formContainer);

// Champ nom
const nameLabel = document.createElement('label');
nameLabel.textContent = 'Nom: ';
formContainer.appendChild(nameLabel);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.id = 'name';
nameInput.style.marginBottom = '10px';
nameInput.style.marginLeft = '5px';
formContainer.appendChild(nameInput);
formContainer.appendChild(document.createElement('br'));

// Champ email
const emailLabel = document.createElement('label');
emailLabel.textContent = 'Email: ';
formContainer.appendChild(emailLabel);

const emailInput = document.createElement('input');
emailInput.type = 'email';
emailInput.id = 'email';
emailInput.style.marginBottom = '10px';
emailInput.style.marginLeft = '5px';
formContainer.appendChild(emailInput);
formContainer.appendChild(document.createElement('br'));

// Champ mot de passe
const passwordLabel = document.createElement('label');
passwordLabel.textContent = 'Mot de passe: ';
formContainer.appendChild(passwordLabel);

const passwordInput = document.createElement('input');
passwordInput.type = 'password';
passwordInput.id = 'password';
passwordInput.style.marginLeft = '5px';
formContainer.appendChild(passwordInput);
formContainer.appendChild(document.createElement('br'));

// Bouton soumettre
const submitButton = document.createElement('button');
submitButton.textContent = 'S\'inscrire';
submitButton.disabled = true;
submitButton.style.marginTop = '15px';
submitButton.style.padding = '8px 16px';
formContainer.appendChild(submitButton);

// Message de validation
const validationMessage = document.createElement('div');
validationMessage.style.marginTop = '10px';
validationMessage.style.color = 'red';
formContainer.appendChild(validationMessage);

// Validation du nom
const name$ = fromEvent(nameInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return {
      value,
      valid: value.length >= 2,
      error: value.length < 2 ? 'Le nom doit contenir au moins 2 caractères' : null,
    };
  }),
  startWith({
    value: '',
    valid: false,
    error: 'Le nom doit contenir au moins 2 caractères',
  })
);

// Validation de l'email
const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
const email$ = fromEvent(emailInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return {
      value,
      valid: emailRegex.test(value),
      error: !emailRegex.test(value)
        ? 'Veuillez entrer une adresse email valide'
        : null,
    };
  }),
  startWith({
    value: '',
    valid: false,
    error: 'Veuillez entrer une adresse email valide',
  })
);

// Validation du mot de passe
const password$ = fromEvent(passwordInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value;
    return {
      value,
      valid: value.length >= 6,
      error: value.length < 6 ? 'Le mot de passe doit contenir au moins 6 caractères' : null,
    };
  }),
  startWith({
    value: '',
    valid: false,
    error: 'Le mot de passe doit contenir au moins 6 caractères',
  })
);

// Combiner l'état de validation de tous les champs
combineLatest([name$, email$, password$])
  .pipe(debounceTime(300))
  .subscribe(([nameState, emailState, passwordState]) => {
    // Vérifier si le formulaire est valide
    const isFormValid =
      nameState.valid && emailState.valid && passwordState.valid;
    submitButton.disabled = !isFormValid;

    // Afficher les messages d'erreur
    if (!isFormValid) {
      const errors = [
        nameState.error,
        emailState.error,
        passwordState.error,
      ].filter((error) => error !== null);

      validationMessage.textContent = errors.join('\n');
    } else {
      validationMessage.textContent = '';
    }
  });

// Événement de clic du bouton soumettre
fromEvent(submitButton, 'click').subscribe(() => {
  const formData = {
    name: nameInput.value,
    email: emailInput.value,
    password: passwordInput.value,
  };

  // Afficher les données du formulaire (en réalité, envoi vers l'API)
  const successMessage = document.createElement('div');
  successMessage.textContent = 'Inscription réussie!';
  successMessage.style.color = 'green';
  successMessage.style.fontWeight = 'bold';
  successMessage.style.marginTop = '10px';
  formContainer.appendChild(successMessage);

  console.log('Données envoyées:', formData);
});

```

## Requêtes simultanées et gestion de l'état de chargement

Un exemple utilisant `forkJoin` pour traiter plusieurs requêtes API en parallèle et rassembler les résultats.

```ts
import {
  forkJoin,
  of,
  throwError,
  Observable,
  ObservableInputTuple,
} from 'rxjs';
import { catchError, delay, finalize } from 'rxjs';

// Définition des interfaces
interface User {
  id: number;
  name: string;
  email: string;
}

interface Post {
  id: number;
  title: string;
  content: string;
}

interface WeatherSuccess {
  city: string;
  temp: number;
  condition: string;
}

interface WeatherError {
  error: string;
}

type Weather = WeatherSuccess | WeatherError;

// Définition du type de résultat
interface ApiResponse {
  user: User;
  posts: Post[];
  weather: Weather;
}

// Création des éléments UI
const apiContainer = document.createElement('div');
apiContainer.innerHTML = '<h3>Exemple de requêtes API multiples:</h3>';
document.body.appendChild(apiContainer);

const loadButton = document.createElement('button');
loadButton.textContent = 'Charger les données';
loadButton.style.padding = '8px 16px';
apiContainer.appendChild(loadButton);

const loadingIndicator = document.createElement('div');
loadingIndicator.style.margin = '10px 0';
loadingIndicator.style.display = 'none';
apiContainer.appendChild(loadingIndicator);

const resultContainer = document.createElement('div');
apiContainer.appendChild(resultContainer);

// Simulation des requêtes API
function fetchUser(id: number): Observable<User> {
  // Requête réussie
  return of({
    id,
    name: `Utilisateur${id}`,
    email: `user${id}@example.com`,
  }).pipe(
    delay(2000) // Délai de 2 secondes
  );
}

function fetchPosts(userId: number): Observable<Post[]> {
  // Requête réussie
  return of([
    { id: 1, title: `Article 1 de ${userId}`, content: 'Contenu...' },
    { id: 2, title: `Article 2 de ${userId}`, content: 'Contenu...' },
  ]).pipe(
    delay(1500) // Délai de 1.5 secondes
  );
}

function fetchWeather(city: string): Observable<WeatherSuccess> {
  // Requête qui échoue parfois
  const shouldFail = Math.random() > 0.7;

  if (shouldFail) {
    return throwError(() => new Error('Échec de récupération des données météo')).pipe(
      delay(1000)
    );
  }

  return of({
    city,
    temp: Math.round(15 + Math.random() * 10),
    condition: ['Ensoleillé', 'Nuageux', 'Pluvieux'][Math.floor(Math.random() * 3)],
  }).pipe(
    delay(1000) // Délai de 1 seconde
  );
}

// Exécuter plusieurs requêtes au clic du bouton
loadButton.addEventListener('click', () => {
  // Réinitialiser l'UI
  resultContainer.innerHTML = '';
  loadingIndicator.style.display = 'block';
  loadingIndicator.textContent = 'Chargement des données...';
  loadButton.disabled = true;

  // Exécuter plusieurs requêtes API simultanément
  forkJoin({
    user: fetchUser(1),
    posts: fetchPosts(1),
    weather: fetchWeather('Paris').pipe(
      // Gestion des erreurs
      catchError((error: Error) => {
        console.error('Erreur API météo:', error);
        return of<WeatherError>({ error: error.message });
      })
    ),
  } as ObservableInputTuple<ApiResponse>)
    .pipe(
      // Traitement à la complétion
      finalize(() => {
        loadingIndicator.style.display = 'none';
        loadButton.disabled = false;
      })
    )
    .subscribe((results: ApiResponse) => {
      // Affichage des informations utilisateur
      const userInfo = document.createElement('div');
      userInfo.innerHTML = `
      <h4>Informations utilisateur</h4>
      <p>Nom: ${results.user.name}</p>
      <p>Email: ${results.user.email}</p>
    `;
      userInfo.style.margin = '10px 0';
      userInfo.style.padding = '10px';
      userInfo.style.backgroundColor = '#f0f0f0';
      userInfo.style.borderRadius = '5px';
      resultContainer.appendChild(userInfo);

      // Affichage des articles
      const postsInfo = document.createElement('div');
      postsInfo.innerHTML = `
      <h4>Liste des articles (${results.posts.length})</h4>
      <ul>
        ${results.posts
          .map((post: { title: string }) => `<li>${post.title}</li>`)
          .join('')}
      </ul>
    `;
      postsInfo.style.margin = '10px 0';
      postsInfo.style.padding = '10px';
      postsInfo.style.backgroundColor = '#f0f0f0';
      postsInfo.style.borderRadius = '5px';
      resultContainer.appendChild(postsInfo);

      // Affichage des informations météo
      const weatherInfo = document.createElement('div');

      if ('error' in results.weather) {
        weatherInfo.innerHTML = `
        <h4>Informations météo</h4>
        <p style="color: red;">Erreur: ${results.weather.error}</p>
      `;
      } else {
        weatherInfo.innerHTML = `
        <h4>Informations météo</h4>
        <p>Ville: ${results.weather.city}</p>
        <p>Température: ${results.weather.temp}°C</p>
        <p>Conditions: ${results.weather.condition}</p>
      `;
      }

      weatherInfo.style.margin = '10px 0';
      weatherInfo.style.padding = '10px';
      weatherInfo.style.backgroundColor = '#f0f0f0';
      weatherInfo.style.borderRadius = '5px';
      resultContainer.appendChild(weatherInfo);
    });
});

```

## Fonctionnalité de recherche annulable

Un exemple combinant `withLatestFrom` et `race` pour implémenter une fonctionnalité de recherche avec timeout et annulation.

```ts
import { fromEvent, timer, race, of, EMPTY } from 'rxjs';
import {
  map,
  debounceTime,
  switchMap,
  tap,
  delay,
  catchError,
  takeUntil,
} from 'rxjs';

// Création de l'UI de recherche
const searchContainer = document.createElement('div');
searchContainer.innerHTML = '<h3>Recherche annulable:</h3>';
document.body.appendChild(searchContainer);

// Champ de recherche
const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Entrez un terme de recherche...';
searchInput.style.padding = '8px';
searchInput.style.width = '250px';
searchContainer.appendChild(searchInput);

// Bouton d'annulation
const cancelButton = document.createElement('button');
cancelButton.textContent = 'Annuler';
cancelButton.style.marginLeft = '10px';
cancelButton.style.padding = '8px 16px';
searchContainer.appendChild(cancelButton);

// Zone de résultats
const resultsContainer = document.createElement('div');
resultsContainer.style.marginTop = '10px';
resultsContainer.style.minHeight = '200px';
resultsContainer.style.padding = '10px';
resultsContainer.style.border = '1px solid #ddd';
resultsContainer.style.borderRadius = '5px';
searchContainer.appendChild(resultsContainer);

// Simulation de requête de recherche
function searchApi(term: string) {
  console.log(`Démarrage de la recherche pour "${term}"...`);

  // Simulation des résultats de recherche
  return of([
    `Résultat de recherche pour "${term}" 1`,
    `Résultat de recherche pour "${term}" 2`,
    `Résultat de recherche pour "${term}" 3`,
  ]).pipe(
    // Délai aléatoire de 2 à 5 secondes
    delay(2000 + Math.random() * 3000),
    // Gestion des erreurs
    catchError((err) => {
      console.error('Erreur de recherche:', err);
      return EMPTY;
    })
  );
}

// Événement d'annulation
const cancel$ = fromEvent(cancelButton, 'click');

// Événement de recherche
const search$ = fromEvent(searchInput, 'input')
  .pipe(
    // Obtenir la valeur d'entrée
    map((event) => (event.target as HTMLInputElement).value.trim()),
    // Attendre 300ms
    debounceTime(300),
    // Ignorer les recherches vides
    tap((term) => {
      if (term === '') {
        resultsContainer.innerHTML = '<p>Veuillez entrer un terme de recherche</p>';
      }
    }),
    // Ne pas traiter les recherches vides
    switchMap((term) => {
      if (term === '') {
        return EMPTY;
      }

      // Afficher le message de recherche en cours
      resultsContainer.innerHTML = '<p>Recherche en cours...</p>';

      // Traitement du timeout (5 secondes)
      const timeout$ = timer(5000).pipe(
        tap(() => console.log('Timeout de la recherche')),
        map(() => ({ type: 'timeout', results: null }))
      );

      // Requête API
      const request$ = searchApi(term).pipe(
        map((results) => ({ type: 'success', results })),
        // Annuler si le bouton d'annulation est pressé
        takeUntil(
          cancel$.pipe(
            tap(() => {
              console.log('Recherche annulée');
              resultsContainer.innerHTML = '<p>Recherche annulée</p>';
            })
          )
        )
      );

      // Le premier entre timeout et complétion de requête
      return race(request$, timeout$);
    })
  )
  .subscribe((response) => {
    if (response.type === 'success') {
      // Recherche réussie
      resultsContainer.innerHTML = '<h4>Résultats de recherche:</h4>';

      if (response.results?.length === 0) {
        resultsContainer.innerHTML += '<p>Aucun résultat trouvé</p>';
      } else {
        const list = document.createElement('ul');
        response.results?.forEach((result) => {
          const item = document.createElement('li');
          item.textContent = result;
          list.appendChild(item);
        });
        resultsContainer.appendChild(list);
      }
    } else if (response.type === 'timeout') {
      // Timeout
      resultsContainer.innerHTML =
        '<p style="color: red;">Timeout de la recherche. Veuillez réessayer.</p>';
    }
  });

```


## Comparaison et guide de sélection des opérateurs de combinaison

Compare les différences entre plusieurs opérateurs de combinaison et aide à choisir selon le cas d'utilisation.

| Opérateur | Timing | Sortie | Cas d'utilisation |
|------------|------------|-----|------------|
| `merge` | Exécution simultanée | Sortie dans l'ordre d'occurrence | Surveillance simultanée d'événements de sources multiples |
| `concat` | Exécution séquentielle | Sortie dans l'ordre | Tâches asynchrones où l'ordre est important |
| `combineLatest` | Nécessite au moins une valeur de toutes les sources | Combinaison de toutes les dernières valeurs | Validation de formulaire |
| `zip` | Nécessite des valeurs à l'index correspondant de toutes les sources | Combinaison des valeurs par index | Synchronisation de données liées |
| `withLatestFrom` | Lors de l'émission de la source principale | Valeur principale et dernières valeurs des autres sources | Combinaison de données auxiliaires |
| `forkJoin` | Quand toutes les sources sont terminées | Dernière valeur de chaque source | Requêtes API multiples |
| `race` | Seule la première source à émettre | Valeurs du flux gagnant uniquement | Timeout, traitement d'annulation |

### Flux de décision pour le choix d'opérateur

1. **Voulez-vous recevoir des valeurs de toutes les sources simultanément?**
   - Oui → `merge`
   - Non → Suivant

2. **Voulez-vous préserver l'ordre des sources?**
   - Oui → `concat`
   - Non → Suivant

3. **Avez-vous besoin de la combinaison des dernières valeurs de chaque source?**
   - Oui → Quand combiner?
     - À chaque nouvelle valeur d'une source → `combineLatest`
     - À chaque valeur d'un flux principal spécifique → `withLatestFrom`
   - Non → Suivant

4. **Avez-vous besoin des valeurs correspondantes par index?**
   - Oui → `zip`
   - Non → Suivant

5. **Avez-vous besoin du résultat après que toutes les sources soient terminées?**
   - Oui → `forkJoin`
   - Non → Suivant

6. **Avez-vous besoin uniquement du plus rapide parmi plusieurs sources alternatives?**
   - Oui → `race`
   - Non → Reconsidérez l'objectif


## Stratégie de commutation

Un exemple de commutation dynamique entre plusieurs sources de données.

```ts
import { fromEvent, merge, interval, of } from 'rxjs';
import { map, switchMap, take, tap } from 'rxjs';

// Création des éléments UI
const switchingContainer = document.createElement('div');
switchingContainer.innerHTML = '<h3>Commutation de source de données:</h3>';
document.body.appendChild(switchingContainer);

// Création des boutons
const source1Button = document.createElement('button');
source1Button.textContent = 'Source 1';
source1Button.style.margin = '5px';
source1Button.style.padding = '5px 10px';
switchingContainer.appendChild(source1Button);

const source2Button = document.createElement('button');
source2Button.textContent = 'Source 2';
source2Button.style.margin = '5px';
source2Button.style.padding = '5px 10px';
switchingContainer.appendChild(source2Button);

const source3Button = document.createElement('button');
source3Button.textContent = 'Source 3';
source3Button.style.margin = '5px';
source3Button.style.padding = '5px 10px';
switchingContainer.appendChild(source3Button);

// Zone d'affichage des résultats
const resultsArea = document.createElement('div');
resultsArea.style.marginTop = '10px';
resultsArea.style.minHeight = '150px';
resultsArea.style.padding = '10px';
resultsArea.style.border = '1px solid #ddd';
resultsArea.style.backgroundColor = '#f9f9f9';
switchingContainer.appendChild(resultsArea);

// 3 sources de données
function createSource1() {
  return interval(1000).pipe(
    take(5),
    map((val) => `Source1: ${val}`),
    tap(() => {
      source1Button.style.backgroundColor = '#c8e6c9';
      source2Button.style.backgroundColor = '';
      source3Button.style.backgroundColor = '';
    })
  );
}

function createSource2() {
  return interval(500).pipe(
    take(8),
    map((val) => `Source2: ${val}`),
    tap(() => {
      source1Button.style.backgroundColor = '';
      source2Button.style.backgroundColor = '#bbdefb';
      source3Button.style.backgroundColor = '';
    })
  );
}

function createSource3() {
  return of('Source3: A', 'Source3: B', 'Source3: C').pipe(
    tap(() => {
      source1Button.style.backgroundColor = '';
      source2Button.style.backgroundColor = '';
      source3Button.style.backgroundColor = '#ffccbc';
    })
  );
}

// Événements de clic des boutons
const source1Click$ = fromEvent(source1Button, 'click').pipe(map(() => 1));

const source2Click$ = fromEvent(source2Button, 'click').pipe(map(() => 2));

const source3Click$ = fromEvent(source3Button, 'click').pipe(map(() => 3));

// Fusionner les clics des boutons
merge(source1Click$, source2Click$, source3Click$)
  .pipe(
    // Commuter vers la source sélectionnée
    switchMap((sourceId) => {
      // Effacer la zone de résultats
      resultsArea.innerHTML = '';

      // Retourner la source sélectionnée
      switch (sourceId) {
        case 1:
          return createSource1();
        case 2:
          return createSource2();
        case 3:
          return createSource3();
        default:
          return of('Aucune source sélectionnée');
      }
    })
  )
  .subscribe((value) => {
    // Afficher le résultat
    const item = document.createElement('div');
    item.textContent = value;
    item.style.padding = '5px';
    item.style.margin = '2px 0';
    item.style.backgroundColor = 'white';
    item.style.borderRadius = '3px';
    resultsArea.appendChild(item);
  });

// Message initial
const initialMessage = document.createElement('div');
initialMessage.textContent =
  'Cliquez sur un bouton pour sélectionner une source de données';
initialMessage.style.color = '#666';
resultsArea.appendChild(initialMessage);

```

## Fusion conditionnelle

Un exemple combinant `merge` et `filter` pour sélectionner des sources de données basées sur des conditions.

```ts
import { merge, interval, fromEvent } from 'rxjs';
import {
  map,
  filter,
  takeUntil,
  withLatestFrom,
  startWith,
} from 'rxjs';

// Création des éléments UI
const conditionalContainer = document.createElement('div');
conditionalContainer.innerHTML = '<h3>Fusion conditionnelle:</h3>';
document.body.appendChild(conditionalContainer);

// Paramètres de filtre
const filterDiv = document.createElement('div');
filterDiv.style.marginBottom = '10px';
conditionalContainer.appendChild(filterDiv);

// Création des cases à cocher
const slowCheck = document.createElement('input');
slowCheck.type = 'checkbox';
slowCheck.id = 'slowCheck';
slowCheck.checked = true;
filterDiv.appendChild(slowCheck);

const slowLabel = document.createElement('label');
slowLabel.htmlFor = 'slowCheck';
slowLabel.textContent = 'Source lente';
slowLabel.style.marginRight = '15px';
filterDiv.appendChild(slowLabel);

const fastCheck = document.createElement('input');
fastCheck.type = 'checkbox';
fastCheck.id = 'fastCheck';
fastCheck.checked = true;
filterDiv.appendChild(fastCheck);

const fastLabel = document.createElement('label');
fastLabel.htmlFor = 'fastCheck';
fastLabel.textContent = 'Source rapide';
fastLabel.style.marginRight = '15px';
filterDiv.appendChild(fastLabel);

const clickCheck = document.createElement('input');
clickCheck.type = 'checkbox';
clickCheck.id = 'clickCheck';
clickCheck.checked = true;
filterDiv.appendChild(clickCheck);

const clickLabel = document.createElement('label');
clickLabel.htmlFor = 'clickCheck';
clickLabel.textContent = 'Événements de clic';
filterDiv.appendChild(clickLabel);

// Bouton d'arrêt
const stopButton = document.createElement('button');
stopButton.textContent = 'Arrêter';
stopButton.style.marginLeft = '15px';
filterDiv.appendChild(stopButton);

// Zone d'affichage des résultats
const conditionalResults = document.createElement('div');
conditionalResults.style.height = '200px';
conditionalResults.style.overflowY = 'auto';
conditionalResults.style.padding = '10px';
conditionalResults.style.border = '1px solid #ddd';
conditionalResults.style.backgroundColor = '#f9f9f9';
conditionalContainer.appendChild(conditionalResults);

// 3 sources de données
// 1. Source lente (toutes les 1 seconde)
const slow$ = interval(1000).pipe(map((val) => ({ type: 'slow', value: val })));

// 2. Source rapide (toutes les 300 millisecondes)
const fast$ = interval(300).pipe(map((val) => ({ type: 'fast', value: val })));

// 3. Événements de clic
const click$ = fromEvent(document.body, 'click').pipe(
  map((event) => ({
    type: 'click',
    value: {
      x: (event as MouseEvent).clientX,
      y: (event as MouseEvent).clientY,
    },
  }))
);

// Surveiller l'état des cases à cocher
const slowEnabled$ = fromEvent(slowCheck, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(true)
);

const fastEnabled$ = fromEvent(fastCheck, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(true)
);

const clickEnabled$ = fromEvent(clickCheck, 'change').pipe(
  map((event) => (event.target as HTMLInputElement).checked),
  startWith(true)
);

// Événement d'arrêt
const stop$ = fromEvent(stopButton, 'click');

// Fusion conditionnelle
merge(
  // Combiner la source lente avec l'état d'activation
  slow$.pipe(
    withLatestFrom(slowEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  ),

  // Combiner la source rapide avec l'état d'activation
  fast$.pipe(
    withLatestFrom(fastEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  ),

  // Combiner la source de clic avec l'état d'activation
  click$.pipe(
    withLatestFrom(clickEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  )
)
  .pipe(takeUntil(stop$))
  .subscribe((event) => {
    // Afficher le résultat
    const item = document.createElement('div');

    switch (event.type) {
      case 'slow':
        item.textContent = `Source lente: ${event.value}`;
        item.style.color = '#1b5e20';
        break;
      case 'fast':
        item.textContent = `Source rapide: ${event.value}`;
        item.style.color = '#0d47a1';
        break;
      case 'click':
        const clickValue = event.value as { x: number; y: number };
        item.textContent = `Clic: X=${clickValue.x}, Y=${clickValue.y}`;
        item.style.color = '#bf360c';
        break;
    }

    item.style.padding = '3px';
    item.style.margin = '2px 0';
    conditionalResults.prepend(item); // Afficher les nouveaux en haut
  });

```

## Résumé du choix des opérateurs de combinaison

| Objectif | Opérateur | Caractéristiques |
|------|--------------|------|
| Synchroniser constamment les dernières valeurs de plusieurs sources | `combineLatest` | Combine toujours les dernières valeurs de chaque Observable |
| Obtenir tout en une fois après que tout soit terminé | `forkJoin` | Émet uniquement la dernière valeur (une seule fois) |
| Traiter synchroniquement dans l'ordre | `zip` | Combine et émet une valeur de chaque Observable |
| Référencer les dernières valeurs des autres au moment du déclenchement | `withLatestFrom` | Attache les dernières valeurs des flux secondaires lors de l'émission du flux principal |

## Résumé

Les opérateurs de combinaison sont des outils puissants pour combiner plusieurs sources de données en un seul flux. En choisissant l'opérateur approprié, vous pouvez exprimer des flux de données asynchrones complexes de manière concise et déclarative.

### Points clés pour maîtriser les opérateurs de combinaison

1. **Choisir selon le cas d'utilisation**: Chaque opérateur est optimisé pour des cas d'utilisation spécifiques. Choisissez l'opérateur approprié selon l'objectif.
2. **Comprendre le timing d'émission**: Le comportement des opérateurs de combinaison dépend fortement du moment où les valeurs sont émises. Comprendre le timing d'émission de chaque opérateur est important.
3. **Considérer la gestion des erreurs**: Réfléchissez au comportement quand une erreur se produit dans une partie du flux combiné (si tout échoue ou si le traitement continue partiellement).
4. **Comprendre les conditions de complétion**: Comprenez quand le flux combiné se termine, et utilisez explicitement `takeUntil` etc. pour terminer si nécessaire.
5. **Utiliser la sécurité de type**: En utilisant TypeScript, vous pouvez gérer les opérateurs de combinaison de manière type-safe. Particulièrement pour les combinaisons complexes, les avantages des types sont importants.

Les opérateurs de combinaison peuvent être utilisés dans de nombreux scénarios pratiques comme le traitement d'événements UI, les requêtes API multiples, et la validation de formulaire. En maîtrisant ces opérateurs, vous pouvez exploiter la vraie puissance de la programmation réactive avec RxJS.

---
Ensuite, passez à [Gestion des erreurs](/fr/guide/error-handling/strategies) pour apprendre à écrire du code RxJS plus robuste!

