from flask import Flask, render_template, request, jsonify
from a_star import a_star_search, get_graph # Our A* and graph stuff

app = Flask(__name__)

# Load our street graph when app starts
get_graph()

@app.route('/')
def home_page():
    # Show the main map page
    return render_template('index.html')

@app.route('/astar_route', methods=['POST'])
def get_astar_route():
    # Get start/end points from web page
    data = request.json
    start_point = tuple(data['start'])
    end_point = tuple(data['end'])

    # Find A* path
    path_data, total_dist, total_time = a_star_search(start_point, end_point)

    if path_data:
        return jsonify({
            'path': path_data,
            'distance': total_dist,
            'time': total_time,
        })
    else:
        return jsonify({'error': 'A* path not found.'}), 400

if __name__ == '__main__':
    app.run(debug=True) # Run our web app