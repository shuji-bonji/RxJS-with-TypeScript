---
description: O operador defer faz com que a função de fábrica do Observable seja atrasada até o ponto de assinatura. Isso é útil quando você deseja avaliar um valor ou processo diferente cada vez que você assina, como o tempo atual, valores aleatórios, solicitações de API dinâmicas ou outros processos cujos resultados mudam no momento da execução.
titleTemplate: ':title | RxJS'
---

# defer - Criação de Observable com avaliação atrasada

O operador `defer` executa a função de fábrica Observable no **ponto de assinatura** e retorna o Observable resultante. Isso permite atrasar a criação de um Observable até que ele seja realmente assinado.

## Sintaxe básica e operação

```ts
import { defer, of } from 'rxjs';

const random$ = defer(() => of(Math.random()));

random$.subscribe(console.log);
random$.subscribe(console.log);

// Saída:
// 0.8727962287400634
// 0.8499299688934545
```

Neste exemplo, `Math.random()` é avaliado para cada assinatura, então um valor diferente é emitido cada vez.

[🌐 Documentação Oficial RxJS - defer](https://rxjs.dev/api/index/function/defer)

## Exemplos de Aplicação Típicos

Isso é útil quando você deseja executar **processos** como APIs, recursos externos, tempo atual, números aleatórios, etc., cujos resultados variam dependendo do momento da execução.

```ts
import { defer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

function fetchUser(userId: number) {
  return defer(() =>
    ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`)
  );
}

fetchUser(1).subscribe(console.log);

// Saída:
// {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
```

## Exemplos de código prático (com UI)

`defer` é especialmente útil para processos que têm efeitos colaterais ou produzem resultados diferentes a cada vez.

No código abaixo, você pode experimentar o que significa usar `defer` para "gerar um Observable diferente cada vez que ele é assinado".
Isso é especialmente útil em casos** onde você deseja fazer o processo de busca **toda vez** em vez de armazená-lo em cache.

### ✅ 1. gerar um número aleatório cada vez
```ts
import { defer, of } from 'rxjs';

// Observable que gera números aleatórios
const randomNumber$ = defer(() => {
  const random = Math.floor(Math.random() * 100);
  return of(random);
});

// Criar elementos de UI
const randomContainer = document.createElement('div');
randomContainer.innerHTML = '<h3>Geração de valor aleatório com defer:</h3>';
document.body.appendChild(randomContainer);

// Botão Gerar
const generateButton = document.createElement('button');
generateButton.textContent = 'Gerar valor aleatório';
randomContainer.appendChild(generateButton);

// Área de exibição de histórico
const randomHistory = document.createElement('div');
randomHistory.style.marginTop = '10px';
randomHistory.style.padding = '10px';
randomHistory.style.border = '1px solid #ddd';
randomHistory.style.maxHeight = '200px';
randomHistory.style.overflowY = 'auto';
randomContainer.appendChild(randomHistory);

// Evento do botão
generateButton.addEventListener('click', () => {
  randomNumber$.subscribe(value => {
    const entry = document.createElement('div');
    entry.textContent = `Valor gerado: ${value}`;
    entry.style.padding = '5px';
    entry.style.margin = '2px 0';
    entry.style.backgroundColor = '#f5f5f5';
    entry.style.borderRadius = '3px';
    randomHistory.insertBefore(entry, randomHistory.firstChild);
  });
});

// Texto de explicação
const randomExplanation = document.createElement('p');
randomExplanation.textContent = 'Cada vez que você clicar no botão "Gerar valor aleatório", um novo valor aleatório será gerado. Se você usar of normal, o valor será gerado apenas uma vez no início, mas usando defer, você pode gerar um novo valor cada vez.';
randomContainer.appendChild(randomExplanation);
```

### ✅ 2. Executar cada solicitação de API

Como `defer` cria um novo Observable cada vez que ele é assinado, é especialmente útil em situações onde você deseja executar diferentes solicitações de API com base em **entrada do usuário, etc.**.
Por exemplo, use o seguinte cenário.

- ✅ Buscar em URLs diferentes dependendo de consultas ou parâmetros dinâmicos
- ✅ Buscar os dados mais recentes cada vez** sem usar o cache
- ✅ Deseja avaliar lentamente o processamento quando um evento ocorre

```ts
import { defer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const container = document.createElement('div');
container.innerHTML = '<h3>Solicitação de API com defer:</h3>';
document.body.appendChild(container);

// Campo de entrada
const input = document.createElement('input');
input.placeholder = 'Digite o ID do usuário';
container.appendChild(input);

// Botão Executar
const button = document.createElement('button');
button.textContent = 'Obter informações do usuário';
container.appendChild(button);

// Exibição de resultado
const resultBox = document.createElement('pre');
resultBox.style.border = '1px solid #ccc';
resultBox.style.padding = '10px';
resultBox.style.marginTop = '10px';
container.appendChild(resultBox);

// Evento do botão
button.addEventListener('click', () => {
  const userId = input.value.trim();
  if (!userId) {
    resultBox.textContent = 'Por favor, digite o ID do usuário';
    return;
  }

  const user$ = defer(() =>
    ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`)
  );

  resultBox.textContent = 'Carregando...';
  user$.subscribe({
    next: (data) => (resultBox.textContent = JSON.stringify(data, null, 2)),
    error: (err) => (resultBox.textContent = `Erro: ${err.message}`),
  });
});
```

Neste exemplo, o `defer` faz com que `ajax.getJSON()` seja chamado quando o usuário pressiona o botão,
**`of(ajax.getJSON(...)) Ao contrário de `defer`, que avalia desde o início, você tem controle completo** sobre o momento da execução.
